#!/usr/bin/env python3
"""wif_check.py - compare iGame3D's own poses with tools/wif.py's BVH of the same animation.

    python3 -I tools/wif_check.py C:/LIB_MCP/ig3d_bones.txt BVHFOLDER

The bones file is written by tools/ig3d_bones.lua running inside iGame3D (igame3dvs.exe):
world positions of every bone at chosen keys. BVHFOLDER holds wif.py's output (use
`wif.py --include-originals --out DIR` to include clips whose original BVH exists).
Both poses are made relative to the hips and scaled to the same leg length (the WIF
models and the Truebones BVH rig differ in size), then compared joint by joint; the
result is the mean joint distance in % of leg length (hip to ankle). Standard library only.
"""
import glob, math, os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bvh  # noqa: E402

JOINTS = ['abdomen', 'chest', 'neck', 'head', 'lShldr', 'lForeArm', 'lHand', 'rShldr', 'rForeArm', 'rHand',
          'lThigh', 'lShin', 'lFoot', 'rThigh', 'rShin', 'rFoot']


def read_engine(path):
    data, cur = {}, None
    for line in open(path, errors='replace').read().splitlines():
        p = line.split(',')
        if p[0] == 'anim':
            cur = re.sub(r'(\.?bvh)\d*$', '', p[2], flags=re.I)
            data[cur] = {}
        elif p[0] == 'key' and cur:
            k, name = int(p[1]), p[2]
            data[cur].setdefault(k, {})['hip' if name == 'None' else name] = tuple(float(v) for v in p[4:7])
    return data


def leg(pose):
    return (math.dist(pose['lThigh'], pose['lShin']) + math.dist(pose['lShin'], pose['lFoot']) +
            math.dist(pose['rThigh'], pose['rShin']) + math.dist(pose['rShin'], pose['rFoot'])) / 2


def norm(pose):
    h, s = pose['hip'], 1 / leg(pose)
    return {n: ((p[0] - h[0]) * s, (p[1] - h[1]) * s, (p[2] - h[2]) * s) for n, p in pose.items()}


def main():
    eng = read_engine(sys.argv[1])
    folder = sys.argv[2]
    worst = 0
    for anim, keys in eng.items():
        f = [p for p in glob.glob(os.path.join(folder, '*.bvh')) if os.path.basename(p)[:-4].lower() == anim.lower()]
        if not f:
            print('%-10s no BVH in %s' % (anim, folder))
            continue
        joints, motion, _ = bvh.parse(open(f[0]).read())
        names = [j['name'] for j in joints]
        errs = []
        for k, ep in sorted(keys.items()):
            if k >= len(motion):
                continue
            pos = bvh.fk(joints, motion[k])
            bp = {n: pos[i] for i, n in enumerate(names) if n}
            a, b = norm(ep), norm(bp)
            e = sum(math.dist(a[j], b[j]) for j in JOINTS) / len(JOINTS)
            errs.append((k, e))
        line = '  '.join('k%d %.1f%%' % (k, e * 100) for k, e in errs)
        mean = sum(e for _, e in errs) / len(errs) * 100
        worst = max(worst, mean)
        print('%-10s mean %.1f%% of leg length   %s' % (anim, mean, line))
    return 0 if worst < 10 else 1


if __name__ == '__main__':
    sys.exit(main())
