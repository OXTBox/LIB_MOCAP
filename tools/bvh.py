"""bvh.py - read a BVH file into joint world positions per frame, with joint names mapped
to LIB_MOCAP's canonical (Mixamo-style) names. Standard library only.

    import bvh
    clip = bvh.load(path)       # {'names', 'parents', 'frames' (list of [(x,y,z)...]), 'fps', 'unmapped'}

Rotation channels are applied in the order the file lists them (BVH convention:
"Zrotation Xrotation Yrotation" means R = Rz * Rx * Ry). Position channels (usually the
root only) are added to the joint's OFFSET. End Sites become joints named after their
parent: head -> HeadTop_End, a foot or heel -> <side>Toe_End, others <name>_End.

Canonical names: Hips Spine Spine1 Spine2 Neck Head HeadTop_End, <side>Shoulder (collar)
<side>Arm <side>ForeArm <side>Hand, <side>UpLeg <side>Leg <side>Foot <side>Heel
<side>ToeBase <side>Toe_End. Names the map does not know keep their own name.
"""
import math, re

# ---------------------------------------------------------------------------- parsing
def parse(text):
    tok = text.split()
    i = 0
    joints = []            # dicts: name, parent, offset, channels, end
    stack = []
    while i < len(tok):
        t = tok[i]
        if t in ('ROOT', 'JOINT'):
            joints.append({'name': tok[i + 1], 'parent': stack[-1] if stack else -1, 'channels': [], 'end': False})
            i += 2
        elif t == 'End':           # End Site
            joints.append({'name': None, 'parent': stack[-1], 'channels': [], 'end': True})
            i += 2
        elif t == '{':
            stack.append(len(joints) - 1)
            i += 1
        elif t == '}':
            stack.pop()
            i += 1
        elif t == 'OFFSET':
            joints[stack[-1]]['offset'] = tuple(float(v) for v in tok[i + 1:i + 4])
            i += 4
        elif t == 'CHANNELS':
            n = int(tok[i + 1])
            joints[stack[-1]]['channels'] = tok[i + 2:i + 2 + n]
            i += 2 + n
        elif t == 'MOTION':
            i += 1
            break
        else:
            i += 1
    # MOTION
    nframes, ftime = 0, 1 / 30
    while i < len(tok):
        if tok[i] == 'Frames:':
            nframes = int(tok[i + 1])
            i += 2
        elif tok[i] == 'Frame' and tok[i + 1] == 'Time:':
            ftime = float(tok[i + 2])
            i += 3
            break
        else:
            i += 1
    nch = sum(len(j['channels']) for j in joints)
    vals = [float(v) for v in tok[i:i + nframes * nch]]
    nframes = len(vals) // nch if nch else 0
    motion = [vals[k * nch:(k + 1) * nch] for k in range(nframes)]
    return joints, motion, ftime


# ---------------------------------------------------------------------------- math
def rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == 'X':
        return ((1, 0, 0), (0, c, -s), (0, s, c))
    if axis == 'Y':
        return ((c, 0, s), (0, 1, 0), (-s, 0, c))
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def mm(a, b):
    return tuple(tuple(sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)) for r in range(3))


def mv(a, v):
    return tuple(sum(a[r][k] * v[k] for k in range(3)) for r in range(3))


I3 = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


def local(joints, frame):
    """per joint: (offset with position channels added, local rotation matrix)"""
    out = []
    c = 0
    for j in joints:
        off = list(j.get('offset', (0, 0, 0)))
        r = I3
        for ch in j['channels']:
            v = frame[c]
            c += 1
            if ch.endswith('position'):
                off['XYZ'.index(ch[0])] += v
            else:
                r = mm(r, rot(ch[0], v))
        out.append((off, r))
    return out


def fk(joints, frame, loc=None):
    """world positions of every joint (End Sites included) for one frame"""
    pos, rots = [], []
    for j, (off, r) in zip(joints, loc if loc is not None else local(joints, frame)):
        p = j['parent']
        if p < 0:
            pos.append(tuple(off))
            rots.append(r)
        else:
            d = mv(rots[p], off)
            pos.append((pos[p][0] + d[0], pos[p][1] + d[1], pos[p][2] + d[2]))
            rots.append(mm(rots[p], r))
    return pos


def m2q(m):
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s)


def q2m(q):
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def slerp(a, b, t):
    d = sum(x * y for x, y in zip(a, b))
    if d < 0:
        b, d = tuple(-x for x in b), -d
    if d > 0.9995:
        q = tuple(x + (y - x) * t for x, y in zip(a, b))
    else:
        th = math.acos(d)
        s0, s1 = math.sin((1 - t) * th) / math.sin(th), math.sin(t * th) / math.sin(th)
        q = tuple(s0 * x + s1 * y for x, y in zip(a, b))
    n = math.sqrt(sum(x * x for x in q)) or 1
    return tuple(x / n for x in q)


def upsample(joints, motion, fps, target):
    """frames at the target rate: rotations slerped, offsets/positions lerped"""
    keys = [local(joints, f) for f in motion]
    quats = [[m2q(r) for _, r in k] for k in keys]
    n = int((len(keys) - 1) * target / fps) + 1
    out = []
    for i in range(n):
        t = i * fps / target
        a = min(int(t), len(keys) - 1)
        b = min(a + 1, len(keys) - 1)
        f = t - a
        loc = []
        for jn in range(len(joints)):
            oa, ob = keys[a][jn][0], keys[b][jn][0]
            loc.append(([x + (y - x) * f for x, y in zip(oa, ob)], q2m(slerp(quats[a][jn], quats[b][jn], f))))
        out.append(fk(joints, None, loc))
    return out


# ---------------------------------------------------------------------------- names
SIDE = {'left': 'Left', 'right': 'Right', 'l': 'Left', 'r': 'Right'}
BODY = {
    'hips': 'Hips', 'hip': 'Hips', 'pelvis': 'Hips', 'root': 'Hips',
    'spine': 'Spine', 'waist': 'Spine', 'abdomen': 'Spine', 'lowerback': 'Spine',
    'spine1': 'Spine1', 'chest': 'Spine2', 'spine2': 'Spine2', 'upperchest': 'Spine2',
    'neck': 'Neck', 'head': 'Head',
}
LIMB = {   # without side; 'shoulder' is decided by whether the rig has collars
    'upleg': 'UpLeg', 'hip': 'UpLeg', 'thigh': 'UpLeg',
    'leg': 'Leg', 'knee': 'Leg', 'shin': 'Leg',
    'foot': 'Foot', 'ankle': 'Foot',
    'footheel': 'Heel', 'heel': 'Heel',
    'toebase': 'ToeBase', 'toe': 'ToeBase',
    'collar': 'Shoulder', 'clavicle': 'Shoulder',
    'arm': 'Arm', 'shldr': 'Arm', 'uparm': 'Arm',
    'forearm': 'ForeArm', 'elbow': 'ForeArm',
    'hand': 'Hand', 'wrist': 'Hand',
}
FINGER = re.compile(r'thumb|index|mid|ring|pinky|finger|carpal', re.I)


def split_side(name):
    m = re.match(r'^(left|right)_?(.+)$', name, re.I) or re.match(r'^([lr])([A-Z].*)$', name)
    if m:
        return SIDE[m.group(1).lower()], m.group(2)
    return None, name


def canonical(names):
    """map a rig's joint names to canonical ones; returns a list (None = drop)"""
    bare = [re.sub(r'^.*:', '', n) for n in names]
    has_collar = any(re.search(r'collar|clavicle', n, re.I) for n in bare)
    out, used = [], set()
    for n in bare:
        side, rest = split_side(n)
        key = rest.lower()
        c = None
        if FINGER.search(n):
            c = None
        elif side:
            if key == 'shoulder':
                c = side + ('Arm' if has_collar else 'Shoulder')
            elif key in LIMB:
                c = side + LIMB[key]
            elif key in ('wrist', ):
                c = side + 'Hand'
            else:
                c = n
        else:
            c = BODY.get(key, n)
        if c in used:          # a second joint wanting the same name keeps its own
            c = n
        used.add(c)
        out.append(c)
    return out


def load(path, min_fps=24, target_fps=30):
    """files slower than min_fps (iGame3D tables are 5 keys a second) are slerped up to
    target_fps; the result says so in 'upsampled_from'"""
    joints, motion, ftime = parse(open(path, encoding='utf-8', errors='replace').read())
    if not motion:
        raise ValueError('no motion data')
    names = canonical([j['name'] or '' for j in joints])
    # End Sites: name after the parent
    for k, j in enumerate(joints):
        if j['end']:
            pn = names[j['parent']] or ''
            side, rest = split_side(pn)
            if pn == 'Head':
                names[k] = 'HeadTop_End'
            elif side and rest in ('Foot', 'Heel', 'ToeBase'):
                names[k] = side + 'Toe_End' if side + 'Toe_End' not in names else pn + '_End'
            else:
                names[k] = pn + '_End'
            if FINGER.search(pn) or pn.endswith('Hand') or (pn.endswith('_End')):
                names[k] = pn + '_End'
    fps = (round(1 / ftime) if abs(1 / ftime - round(1 / ftime)) < 0.05 else round(1 / ftime, 3)) if ftime else 30
    upsampled = None
    if fps < min_fps and len(motion) > 1:
        frames = upsample(joints, motion, fps, target_fps)
        upsampled, fps = fps, target_fps
    else:
        frames = [fk(joints, f) for f in motion]
    known = set(BODY.values()) | {s + v for s in ('Left', 'Right') for v in LIMB.values()} | \
        {'HeadTop_End', 'LeftToe_End', 'RightToe_End'}
    return {'names': names, 'parents': [j['parent'] for j in joints], 'frames': frames,
            'fps': fps, 'upsampled_from': upsampled,
            'unmapped': [n for n, j in zip(names, joints) if n not in known and not j['end']]}
