#!/usr/bin/env python3
"""mocap_skin.py - put a procedural, skinned body on an animated skeleton GLB
(MoCapAnything's *_skeleton.glb: joints j0..jN as plain nodes, rigid "bone_N" stick meshes,
one animation, no skin), and write files xt3d and Vibecode3D can play.

    python3 -I mocap_skin.py SKELETON.glb [--out OUTBASE] [--fps 30] [--thick 1.0]

Writes
    OUTBASE_skinned.glb   one skinned mesh + skin + the animation (raylib / xt3d:
                          xt3d_LoadModel + xt3d_LoadModelAnimations + xt3d_UpdateModelAnimation)
    OUTBASE_clip.txt      plain text for xTalk / Vibecode3D: joint names and parents, then one
                          line per frame of joint world positions (x,y,z per joint, metres, Y up)

The body: an octagonal prism for every bone (parent joint -> child joint), thickness by body
part, plus a head. Each prism's vertices are weighted to the parent joint, blending toward the
child joint near the far end, so elbows and knees bend smoothly instead of cracking.
Vertex colours: left side blue, right side red, centre white (vector-style).

--fps   MoCapAnything's GLB stores 145 keys over 9.6 s while its BVH says 30 fps; give the
        real capture rate to retime the keys (0 = keep the GLB's times).
Standard library only.
"""
import argparse, json, math, os, re, struct, sys

# ---------------------------------------------------------------------------- math (column-major 4x4 as flat lists, glTF order)
def quat_mat(q, t=(0, 0, 0), s=(1, 1, 1)):
    x, y, z, w = q
    m = [1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0,
         2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0,
         2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0,
         t[0], t[1], t[2], 1]
    for c in range(3):
        for r in range(3):
            m[c * 4 + r] *= s[c]
    return m


def mul(a, b):
    out = [0.0] * 16
    for c in range(4):
        for r in range(4):
            out[c * 4 + r] = sum(a[k * 4 + r] * b[c * 4 + k] for k in range(4))
    return out


def inv_rigid(m):
    """inverse of rotation(+uniform scale)+translation"""
    r = [[m[c * 4 + rr] for c in range(3)] for rr in range(3)]       # r[row][col]
    s2 = sum(r[i][0] ** 2 for i in range(3)) or 1.0
    rt = [[r[c][rr] / s2 for c in range(3)] for rr in range(3)]       # transpose / scale^2
    t = [m[12], m[13], m[14]]
    ti = [-sum(rt[rr][k] * t[k] for k in range(3)) for rr in range(3)]
    return [rt[0][0], rt[1][0], rt[2][0], 0, rt[0][1], rt[1][1], rt[2][1], 0,
            rt[0][2], rt[1][2], rt[2][2], 0, ti[0], ti[1], ti[2], 1]


def pos(m):
    return (m[12], m[13], m[14])


def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def scl(a, k): return (a[0] * k, a[1] * k, a[2] * k)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def norm(a):
    l = math.sqrt(dot(a, a)) or 1.0
    return (a[0] / l, a[1] / l, a[2] / l)


def slerp_free_lerp(q0, q1, f):
    if dot4(q0, q1) < 0:
        q1 = [-v for v in q1]
    q = [a + (b - a) * f for a, b in zip(q0, q1)]
    l = math.sqrt(sum(v * v for v in q)) or 1.0
    return [v / l for v in q]


def dot4(a, b): return sum(x * y for x, y in zip(a, b))

# ---------------------------------------------------------------------------- GLB io
COMP = {5120: 'b', 5121: 'B', 5122: 'h', 5123: 'H', 5125: 'I', 5126: 'f'}
NCOMP = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def read_glb(path):
    f = open(path, 'rb').read()
    if f[:4] != b'glTF':
        sys.exit('not a GLB: ' + path)
    jl = struct.unpack('<I', f[12:16])[0]
    g = json.loads(f[20:20 + jl])
    off = 20 + jl
    bin_ = b''
    if off < len(f):
        bl = struct.unpack('<I', f[off:off + 4])[0]
        bin_ = f[off + 8:off + 8 + bl]
    return g, bin_


def accessor(g, bin_, i):
    a = g['accessors'][i]
    bv = g['bufferViews'][a['bufferView']]
    n, c = NCOMP[a['type']], COMP[a['componentType']]
    size = struct.calcsize('<' + c)
    stride = bv.get('byteStride') or n * size
    base = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
    out = []
    for k in range(a['count']):
        v = struct.unpack_from('<%d%s' % (n, c), bin_, base + k * stride)
        out.append(list(v) if n > 1 else v[0])
    return out


class Writer:
    def __init__(self):
        self.bin = bytearray()
        self.views, self.accs = [], []

    def add(self, fmt, values, gltype, ctype, target=None, minmax=False):
        while len(self.bin) % 4:
            self.bin.append(0)
        start = len(self.bin)
        flat = []
        for v in values:
            flat.extend(v if isinstance(v, (list, tuple)) else [v])
        self.bin += struct.pack('<%d%s' % (len(flat), fmt), *flat)
        view = {'buffer': 0, 'byteOffset': start, 'byteLength': len(self.bin) - start}
        if target:
            view['target'] = target
        self.views.append(view)
        acc = {'bufferView': len(self.views) - 1, 'componentType': ctype, 'count': len(values), 'type': gltype}
        if minmax:
            n = NCOMP[gltype]
            cols = [[(v if isinstance(v, (list, tuple)) else [v])[i] for v in values] for i in range(n)]
            acc['min'] = [min(c) for c in cols]
            acc['max'] = [max(c) for c in cols]
        self.accs.append(acc)
        return len(self.accs) - 1


def write_glb(path, g, bin_):
    while len(bin_) % 4:
        bin_ += b'\0'
    js = json.dumps(g, separators=(',', ':')).encode('utf-8')
    while len(js) % 4:
        js += b' '
    total = 12 + 8 + len(js) + 8 + len(bin_)
    with open(path, 'wb') as f:
        f.write(struct.pack('<4sII', b'glTF', 2, total))
        f.write(struct.pack('<I4s', len(js), b'JSON') + js)
        f.write(struct.pack('<I4s', len(bin_), b'BIN\0') + bytes(bin_))

# ---------------------------------------------------------------------------- skeleton
MIXAMO_65 = ('Hips Spine Spine1 Spine2 Neck Head HeadTop_End LeftShoulder LeftArm LeftForeArm LeftHand '
             + ' '.join('LeftHand%s%d' % (f, i) for f in ('Thumb', 'Index', 'Middle', 'Ring', 'Pinky') for i in range(1, 5))
             + ' RightShoulder RightArm RightForeArm RightHand '
             + ' '.join('RightHand%s%d' % (f, i) for f in ('Thumb', 'Index', 'Middle', 'Ring', 'Pinky') for i in range(1, 5))
             + ' LeftUpLeg LeftLeg LeftFoot LeftToeBase LeftToe_End RightUpLeg RightLeg RightFoot RightToeBase RightToe_End').split()


def bvh_names(glb_path):
    """joint names from a BVH beside the GLB, in file order (= j0..jN order)"""
    folder = os.path.dirname(os.path.abspath(glb_path))
    for f in sorted(os.listdir(folder)):
        if f.lower().endswith('.bvh'):
            names = re.findall(r'^\s*(?:ROOT|JOINT)\s+(\S+)', open(os.path.join(folder, f), errors='replace').read(), re.M)
            return [n.split(':')[-1] for n in names], f
    return None, None


def load_skeleton(g):
    joints = [i for i, n in enumerate(g['nodes']) if re.fullmatch(r'j\d+', n.get('name', ''))]
    joints.sort(key=lambda i: int(g['nodes'][i]['name'][1:]))
    parent = {}
    for i, n in enumerate(g['nodes']):
        for c in n.get('children', []):
            parent[c] = i
    return joints, parent


def local_matrix(node, rot=None, tr=None):
    q = rot if rot is not None else node.get('rotation', [0, 0, 0, 1])
    t = tr if tr is not None else node.get('translation', [0, 0, 0])
    return quat_mat(q, t, node.get('scale', [1, 1, 1]))


def world_matrices(g, joints, parent, pose=None):
    """pose: {node: (rot, tr)} overrides; returns {node: world matrix}"""
    pose = pose or {}
    out = {}

    def w(i):
        if i in out:
            return out[i]
        r, t = pose.get(i, (None, None))
        m = local_matrix(g['nodes'][i], r, t)
        if i in parent and parent[i] in set(joints):
            m = mul(w(parent[i]), m)
        out[i] = m
        return m
    for j in joints:
        w(j)
    return out


def radius_for(name):
    n = name.lower()
    if 'hand' in n and any(f in n for f in ('thumb', 'index', 'middle', 'ring', 'pinky')):
        return 0.008
    for key, r in (('head', 0.0), ('neck', 0.045), ('spine', 0.11), ('hips', 0.11), ('shoulder', 0.05),
                   ('forearm', 0.035), ('arm', 0.045), ('hand', 0.03), ('upleg', 0.075), ('leg', 0.055),
                   ('foot', 0.04), ('toe', 0.03)):
        if key in n:
            return r
    return 0.03


def side_color(name):
    n = name.lower()
    if n.startswith('left'):
        return (80, 160, 255, 255)
    if n.startswith('right'):
        return (255, 90, 80, 255)
    return (235, 235, 235, 255)

# ---------------------------------------------------------------------------- body
def build_body(joints, names, parent, rest, thick, sides=8):
    """vertices (pos, normal, color, joints4, weights4) and triangle indices, in rest pose"""
    idx_of = {n: k for k, n in enumerate(joints)}
    V, I = [], []

    def ring(center, axis, radius, u, v, color, jw):
        start = len(V)
        for s in range(sides):
            a = 2 * math.pi * s / sides
            d = add(scl(u, math.cos(a)), scl(v, math.sin(a)))
            V.append((add(center, scl(d, radius)), d, color, jw))
        return start

    for j in joints:
        p = parent.get(j)
        if p is None or p not in idx_of:
            continue
        a, b = pos(rest[p]), pos(rest[j])
        axis = sub(b, a)
        length = math.sqrt(dot(axis, axis))
        if length < 1e-5:
            continue
        axis = norm(axis)
        pname = names[idx_of[p]]
        r = radius_for(pname) * thick
        if r <= 0:
            continue
        helper = (0, 1, 0) if abs(axis[1]) < 0.9 else (1, 0, 0)
        u = norm(cross(axis, helper))
        v = cross(axis, u)
        col = side_color(pname)
        pi, ci = idx_of[p], idx_of[j]
        # three rings: start (all parent), middle (all parent), end (blend toward child)
        rings = [ring(a, axis, r, u, v, col, ((pi, ci), (1.0, 0.0))),
                 ring(add(a, scl(axis, length * 0.7)), axis, r * 0.95, u, v, col, ((pi, ci), (1.0, 0.0))),
                 ring(b, axis, r * 0.85, u, v, col, ((pi, ci), (0.5, 0.5)))]
        for r0, r1 in zip(rings, rings[1:]):
            for s in range(sides):
                s1 = (s + 1) % sides
                I += [r0 + s, r1 + s, r1 + s1, r0 + s, r1 + s1, r0 + s1]
        for cap, flip, center, n_ in ((rings[0], True, a, scl(axis, -1)), (rings[-1], False, b, axis)):
            c = len(V)
            V.append((center, n_, col, V[cap][3]))
            for s in range(sides):
                s1 = (s + 1) % sides
                I += [c, cap + s1, cap + s] if flip else [c, cap + s, cap + s1]

    # head: a sphere on the Head joint, weighted fully to it
    if 'Head' in names:
        h = joints[names.index('Head')]
        top = names.index('HeadTop_End') if 'HeadTop_End' in names else None
        c0 = pos(rest[h])
        hr = 0.1 * thick
        if top is not None:
            hr = max(0.06, math.dist(c0, pos(rest[joints[top]])) * 0.55) * thick
        center = add(c0, (0, hr * 0.8, 0))
        rings_n, segs = 8, 12
        start = len(V)
        jw = ((idx_of[h], idx_of[h]), (1.0, 0.0))
        for ri in range(rings_n + 1):
            th = math.pi * ri / rings_n
            for si in range(segs):
                ph = 2 * math.pi * si / segs
                d = (math.sin(th) * math.cos(ph), math.cos(th), math.sin(th) * math.sin(ph))
                V.append((add(center, scl(d, hr)), d, (235, 235, 235, 255), jw))
        for ri in range(rings_n):
            for si in range(segs):
                a0 = start + ri * segs + si
                a1 = start + ri * segs + (si + 1) % segs
                b0, b1 = a0 + segs, a1 + segs
                I += [a0, a1, b1, a0, b1, b0]
    return V, I

# ---------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('glb')
    ap.add_argument('--out', help='output base path (default: beside the input, without _skeleton)')
    ap.add_argument('--fps', type=float, default=30, help='capture rate to retime keys (0 = keep GLB times)')
    ap.add_argument('--thick', type=float, default=1.0, help='body thickness multiplier')
    a = ap.parse_args(argv)
    info = convert(a.glb, a.out, a.fps, a.thick)
    print('joints: %(joints)d (names from %(names_from)s); frames: %(frames)d; GLB key rate %(glb_rate).1f fps -> %(fps)s fps' % info)
    print('body: %(vertices)d vertices, %(triangles)d triangles' % info)
    print('wrote %(glb_out)s and %(clip_out)s' % info)


def convert(glb, out=None, fps=30, thick=1.0):
    """skeleton GLB -> OUT_skinned.glb + OUT_clip.txt; returns a dict of facts"""
    class A: pass
    a = A()
    a.glb, a.out, a.fps, a.thick = glb, out, fps, thick
    g, bin_ = read_glb(a.glb)
    joints, parent = load_skeleton(g)
    if not joints:
        sys.exit('no joint nodes named j0, j1, ... in ' + a.glb)
    names, bvh = bvh_names(a.glb)
    if not names or len(names) != len(joints):
        names = MIXAMO_65 if len(joints) == 65 else ['j%d' % k for k in range(len(joints))]
        src = 'Mixamo 65-joint order' if len(joints) == 65 else 'j-numbers'
    else:
        src = bvh
    rest = world_matrices(g, joints, parent)

    V, I = build_body(joints, names, parent, rest, a.thick)

    # animation: read every channel, retime
    anim = g['animations'][0]
    chans = {}
    times = None
    for ch in anim['channels']:
        s = anim['samplers'][ch['sampler']]
        t_in = accessor(g, bin_, s['input'])
        vals = accessor(g, bin_, s['output'])
        chans[(ch['target']['node'], ch['target']['path'])] = vals
        times = t_in if times is None or len(t_in) > len(times) else times
    nframes = len(times)
    glb_rate = (nframes - 1) / (times[-1] - times[0]) if times[-1] > times[0] else 0
    new_times = [k / a.fps for k in range(nframes)] if a.fps else times

    out = a.out or re.sub(r'_skeleton$', '', os.path.splitext(a.glb)[0])
    w = Writer()
    jset = set(joints)
    # nodes: keep the joints (same names, renamed to real names), drop the stick meshes, add the body
    nodes = []
    remap = {}
    for k, j in enumerate(joints):
        remap[j] = k
    for k, j in enumerate(joints):
        n = g['nodes'][j]
        node = {'name': names[k]}
        for key in ('translation', 'rotation', 'scale'):
            if key in n:
                node[key] = n[key]
        kids = [remap[c] for c in n.get('children', []) if c in jset]
        if kids:
            node['children'] = kids
        nodes.append(node)
    roots = [remap[j] for j in joints if parent.get(j) not in jset]

    P = w.add('f', [v[0] for v in V], 'VEC3', 5126, 34962, minmax=True)
    N = w.add('f', [v[1] for v in V], 'VEC3', 5126, 34962)
    C = w.add('B', [v[2] for v in V], 'VEC4', 5121, 34962)
    w.accs[C]['normalized'] = True
    J = w.add('H', [[v[3][0][0], v[3][0][1], 0, 0] for v in V], 'VEC4', 5123, 34962)
    W = w.add('f', [[v[3][1][0], v[3][1][1], 0, 0] for v in V], 'VEC4', 5126, 34962)
    IDX = w.add('I' if len(V) > 65535 else 'H', I, 'SCALAR', 5125 if len(V) > 65535 else 5123, 34963)
    IBM = w.add('f', [inv_rigid(rest[j]) for j in joints], 'MAT4', 5126)

    samplers, channels = [], []
    T = w.add('f', new_times, 'SCALAR', 5126, minmax=True)
    for (node, path), vals in sorted(chans.items()):
        if node not in jset:
            continue
        O = w.add('f', vals, 'VEC4' if path == 'rotation' else 'VEC3', 5126)
        samplers.append({'input': T, 'output': O, 'interpolation': 'LINEAR'})
        channels.append({'sampler': len(samplers) - 1, 'target': {'node': remap[node], 'path': path}})

    body = len(nodes)
    nodes.append({'name': 'body', 'mesh': 0, 'skin': 0})
    gl = {
        'asset': {'version': '2.0', 'generator': 'LIB_MOCAP mocap_skin.py'},
        'scene': 0, 'scenes': [{'nodes': roots + [body]}], 'nodes': nodes,
        'meshes': [{'name': 'body', 'primitives': [{'attributes': {'POSITION': P, 'NORMAL': N, 'COLOR_0': C,
                                                                   'JOINTS_0': J, 'WEIGHTS_0': W},
                                                    'indices': IDX, 'material': 0}]}],
        'materials': [{'name': 'body', 'pbrMetallicRoughness': {'baseColorFactor': [1, 1, 1, 1],
                                                                 'metallicFactor': 0, 'roughnessFactor': 1}}],
        'skins': [{'name': 'mocap', 'joints': list(range(len(joints))), 'inverseBindMatrices': IBM,
                   'skeleton': roots[0]}],
        'animations': [{'name': os.path.basename(out), 'samplers': samplers, 'channels': channels}],
        'accessors': w.accs, 'bufferViews': w.views, 'buffers': [{'byteLength': len(w.bin)}],
    }
    write_glb(out + '_skinned.glb', gl, w.bin)

    # plain-text clip for xTalk / Vibecode3D
    lines = ['# LIB_MOCAP clip: joint world positions per frame (metres, Y up)',
             'source,%s' % os.path.basename(a.glb), 'fps,%g' % (a.fps or glb_rate), 'frames,%d' % nframes,
             'joints,%d' % len(joints),
             'names,' + ','.join(names),
             'parents,' + ','.join(str(remap[parent[j]]) if parent.get(j) in jset else '-1' for j in joints),
             '# frame lines: x,y,z for joint 0, then joint 1, ...']
    for f in range(nframes):
        pose = {}
        for j in joints:
            r = chans.get((j, 'rotation'))
            t = chans.get((j, 'translation'))
            pose[j] = (r[f] if r else None, t[f] if t else None)
        wm = world_matrices(g, joints, parent, pose)
        lines.append(','.join('%.4f' % c for j in joints for c in pos(wm[j])))
    with open(out + '_clip.txt', 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')

    return {'joints': len(joints), 'names_from': src, 'frames': nframes, 'glb_rate': glb_rate,
            'fps': a.fps or round(glb_rate, 3), 'vertices': len(V), 'triangles': len(I) // 3,
            'glb_out': out + '_skinned.glb', 'clip_out': out + '_clip.txt',
            'root_translation_keys': (joints[0], 'translation') in chans}


if __name__ == '__main__':
    main()
