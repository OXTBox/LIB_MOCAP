#!/usr/bin/env python3
"""wtf_anim.py - the animations inside iGame3D model files (.wtf, the 2008 DeadDays models and
any other with its own bones) as BVH files (TODO A-85; the owner: go forward if they convert).

    python3 -I tools/wtf_anim.py MODEL.wtf... [--out DIR] [--fps 30]
    python3 -I tools/wtf_anim.py --deaddays              all animated DeadDays models
    python3 -I tools/wtf_anim.py --check MODEL.wtf       print the rig and the animations only

wif.py does the Truebones-rig tables (20 bones, a key every 0.2 s). These models are
different: each has its OWN rig and its keys are timed in seconds. Read from the file, decoded
2026-11-15 against officer.wtf's geometry:

  #Bones:       n, then n names, then n pairs  "j,p"
                  j = the bone's joint: index (0-based) into #Vertices
                  p = parent bone number + 1 (1 = hangs from the root point, vertex 0)
                (officer: LLgL's joint is its knee, vertex 1; its parent is 3 - 1 = LegL, the hip;
                 ArmR's pair is "394,12": joint vertex 394 = (-92,646), the mirror of ArmL's
                 shoulder, parent 11 = ches.)
  #Vertices:    x,y,z,bone (bone 0 = not skinned; the joint points are among these)
  #Animations:  count; then per animation: name, key count, and per key
                "start,end" (seconds) and "morph=b,x,y,z,rx,ry,rz" (blend from the bone's
                pose at start to this pose at end) or "set=..." (jump at start).
                b counts bones from 1, in #Bones order; b = 0 is the whole model (the
                root: "Die" drops and tips it), written as the ROOT's position and rotation.

BVH written: a ROOT at the model origin (6 channels: the b = 0 keys; the model turns about its
feet, checked on "Die"), each bone a JOINT at its joint point
(OFFSET = joint - parent joint), 3 rotation channels "Zrotation Yrotation Xrotation" holding
rz, ry, rx (iGame3D's Euler order as wif.py fitted it on the engine's own tables: R = Rz Ry Rx;
**a guess for these models until one is seen moving next to the 2008 game**). Leaf bones get an
End Site at their farthest skinned vertex. Key translations x,y,z are counted and reported
(BVH joints below the root have none). Units are the model's own (officer is about 800 tall).
The root's own moves are tiny in the files (under 2 units); written as they are (their unit is
not known: wif.py found the Truebones tables scaled by 0.005263, but scaling these by that would
throw a dying officer 280 units into the air).
Standard library only.
"""
import glob, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECTS = os.path.dirname(os.path.dirname(HERE))
DEADDAYS = os.path.join(PROJECTS, 'LIB_GAMES', 'igame3D_2008_Texts', 'WTF', 'DeadDays')
OUT = os.path.join(PROJECTS, 'LIB_BVH', 'iGame3D_DeadDays')


def lines_of(path):
    t = open(path, 'rb').read().decode('latin-1')
    return t.replace('\r\n', '\n').replace('\r', '\n').split('\n')


def section(L, name):
    for i, line in enumerate(L):
        if line.strip().rstrip(':') == '#' + name:
            return i
    raise ValueError('no #%s section' % name)


def read_model(path):
    """(rig, animations): rig = dict(names, joints [(x,y,z)], parent [-1 = root], root (x,y,z), tips)"""
    L = lines_of(path)
    b = section(L, 'Bones')
    n = int(L[b + 1])
    names = [L[b + 2 + k].strip() for k in range(n)]
    pairs = [tuple(int(x) for x in L[b + 2 + n + k].split(',')) for k in range(n)]
    v = section(L, 'Vertices')
    nv = int(L[v + 1])
    V = [[float(x) for x in L[v + 2 + k].split(',')] for k in range(nv)]
    joints = [tuple(V[j][:3]) for j, _ in pairs]
    parent = [p - 2 for _, p in pairs]          # p = parent bone number + 1; 1 = the root point
    if any(not -1 <= q < n for q in parent):
        raise ValueError('a parent number is out of range: %r' % (pairs,))
    tips = {}
    for k in range(n):
        if k in parent:
            continue
        mine = [x[:3] for x in V if int(x[3]) == k + 1]
        if mine:
            tips[k] = max(mine, key=lambda q: sum((q[i] - joints[k][i]) ** 2 for i in range(3)))
    # the whole model turns about its origin (its feet), not about vertex 0: officer's "Die" ends
    # at rx -90 with almost no move, which lies on the floor only when the pivot is the origin
    rig = dict(names=names, joints=joints, parent=parent, root=(0.0, 0.0, 0.0), root_point=tuple(V[0][:3]), tips=tips)
    return rig, read_animations(L[section(L, 'Animations'):], n)


def read_animations(L, nbones):
    """[(name, [(start, end, kind, bone 0-based or -1 for the root, [x,y,z,rx,ry,rz])])]"""
    out, i = [], 1
    try:
        count = int(L[i])
    except (IndexError, ValueError):
        return out
    i += 1
    for _ in range(count):
        if i + 1 >= len(L):
            break
        name, nkeys = L[i].strip(), int(L[i + 1])
        i += 2
        keys = []
        for _ in range(nkeys):
            m = re.match(r'^\s*([-\d.eE]+),([-\d.eE]+)\s*$', L[i])
            k = re.match(r'^(morph|set)=(\d+),(.*)$', L[i + 1].strip())
            if not (m and k):
                raise ValueError('animation %s: a key at line %d is not "start,end" + morph/set' % (name, i))
            vals = [float(x) for x in k.group(3).split(',')]
            bone = int(k.group(2)) - 1            # -1 = the whole model (the root)
            if not -1 <= bone < nbones or len(vals) != 6:
                raise ValueError('animation %s: bad key %r' % (name, L[i + 1]))
            keys.append((float(m.group(1)), float(m.group(2)), k.group(1), bone, vals))
            i += 2
        out.append((name, keys))
    return out


def sample(keys, nbones, fps):
    """frames of per-bone [x,y,z,rx,ry,rz] (the root last, at index nbones), fps a second"""
    end = max([k[1] for k in keys] + [0.0])
    nframes = max(1, int(round(end * fps)) + 1)
    per = [sorted([k for k in keys if k[3] == b], key=lambda k: k[0]) for b in list(range(nbones)) + [-1]]
    frames = []
    for f in range(nframes):
        t = f / float(fps)
        pose = []
        for b in range(nbones + 1):
            cur = [0.0] * 6
            for s, e, kind, _, vals in per[b]:
                if t < s:
                    break
                if kind == 'set' or e <= s or t >= e:
                    cur = list(vals)
                else:
                    a = (t - s) / (e - s)
                    cur = [c + (v - c) * a for c, v in zip(cur, vals)]
            pose.append(cur)
        frames.append(pose)
    return frames


def bvh_text(rig, frames, fps):
    names, joints, parent, root = rig['names'], rig['joints'], rig['parent'], rig['root']
    kids = {q: [k for k in range(len(names)) if parent[k] == q] for q in range(-1, len(names))}
    out = ['HIERARCHY', 'ROOT root', '{', '\tOFFSET %.6f %.6f %.6f' % root,
           '\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation']
    order = []

    def emit(k, depth):
        pad = '\t' * depth
        base = joints[parent[k]] if parent[k] >= 0 else root
        off = tuple(joints[k][i] - base[i] for i in range(3))
        out.extend([pad + 'JOINT ' + re.sub(r'\W', '_', names[k]), pad + '{',
                    pad + '\tOFFSET %.6f %.6f %.6f' % off, pad + '\tCHANNELS 3 Zrotation Yrotation Xrotation'])
        order.append(k)
        for c in kids[k]:
            emit(c, depth + 1)
        if not kids[k]:
            tip = rig['tips'].get(k, joints[k])
            out.extend([pad + '\tEnd Site', pad + '\t{',
                        pad + '\t\tOFFSET %.6f %.6f %.6f' % tuple(tip[i] - joints[k][i] for i in range(3)), pad + '\t}'])
        out.append(pad + '}')
    for k in kids[-1]:
        emit(k, 1)
    out += ['}', 'MOTION', 'Frames: %d' % len(frames), 'Frame Time: %.6f' % (1.0 / fps)]
    for pose in frames:
        x, y, z, rx, ry, rz = pose[-1]                  # the whole model
        vals = [x, y, z, rz, ry, rx]                    # the OFFSET already holds the root point
        for k in order:
            x, y, z, rx, ry, rz = pose[k]
            vals += [rz, ry, rx]
        out.append(' '.join('%.4f' % v for v in vals))
    return '\n'.join(out) + '\n'


def convert(path, out_dir, fps=30, write=True):
    rig, anims = read_model(path)
    model = os.path.splitext(os.path.basename(path))[0]
    report = []
    for name, keys in anims:
        moved = sum(1 for k in keys if k[3] >= 0 and any(abs(v) > 1e-9 for v in k[4][:3]))
        frames = sample(keys, len(rig['names']), fps)
        fname = '%s_%s.bvh' % (model, re.sub(r'\W+', '_', name).strip('_') or 'anim')
        if write:
            os.makedirs(out_dir, exist_ok=True)
            open(os.path.join(out_dir, fname), 'w').write(bvh_text(rig, frames, fps))
        report.append((fname, len(keys), len(frames), moved))
    return rig, report


def main(argv):
    if not argv or argv[0] in ('-h', '--help'):
        print(__doc__)
        return 0
    out, fps, check, paths = OUT, 30, False, []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == '--out':
            out, i = argv[i + 1], i + 1
        elif a == '--fps':
            fps, i = int(argv[i + 1]), i + 1
        elif a == '--check':
            check = True
        elif a == '--deaddays':
            paths += sorted(glob.glob(os.path.join(DEADDAYS, '*.wtf')))
        else:
            paths.append(a)
        i += 1
    total = 0
    for p in paths:
        try:
            rig, rep = convert(p, out, fps, write=not check)
        except (ValueError, IndexError) as e:
            print('%s: skipped (%s)' % (os.path.basename(p), e))
            continue
        if not rep:
            continue
        print('%s: %d bones (%s), %d animations' % (os.path.basename(p), len(rig['names']), ' '.join(rig['names']), len(rep)))
        for fname, nk, nf, moved in rep:
            print('   %-40s %3d keys %4d frames%s' % (fname, nk, nf, '  (%d keys move a joint: not in BVH)' % moved if moved else ''))
        total += len(rep)
    print('%d animations %s %s' % (total, 'found (nothing written)' if check else 'written to', '' if check else out))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
