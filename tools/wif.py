#!/usr/bin/env python3
"""wif.py - turn iGame3D animation tables back into BVH files.

    python3 -I tools/wif.py [--out input/BVH/iGame3D] [--include-originals]

In 2008 iGame3D converted the Truebones BVH library (20-bone Poser-style rig) into its own
tables, kept in two places that hold the same data:
  input/BVH/Conversions/*/*.BVH.txt   one animation per file
  input/WIF/*.wtf                     iGame3D models with many animations each (#Animations)
A table is: name, key count, then per key and bone "<frame>,0.1" and
  morph=bone,tx,ty,tz,rx,ry,rz
bone 0..19 in Truebones BVH order (hip first); t = BVH position * 0.005263 (root only);
r = iGame3D's own Euler angles in degrees.

The rotation convention was fitted against the 16 tables whose original Truebones BVH is
still in input/BVH/FreeBVH: R = Rz * Ry * Rx (BVH "Zrotation Yrotation Xrotation"), all
signs positive; mean joint error about 1% of body height. The tables keep every 6th
frame of the 30 fps originals, so a key is 0.2 s (the "0.1" in the file is not the timing).

Writes one BVH per animation with the Truebones hierarchy below. Skips animations whose
original BVH is already in input/BVH (they are better) unless --include-originals.
Standard library only.
"""
import argparse, glob, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
INPUT = os.path.join(ROOT, 'input')
SCALE = 0.005263
KEY_TIME = 0.2

# Truebones rig (from FreeBVH/wall.BVH): name, parent index, offset; End Sites after
BONES = [
    ('hip', -1, (0, 0, 0)),
    ('abdomen', 0, (0, 0, 0)),
    ('chest', 1, (0, 4.862840, -1.823974)),
    ('neckDummy', 2, (0, 8.059053, 0.760605)),
    ('neck', 3, (0, 2.209834, -0.380644)),
    ('head', 4, (0, 3.388650, 0.513082)),
    ('lCollar', 2, (0.580691, 8.059053, 0.760605)),
    ('lShldr', 6, (6.222462, 0.009832, -0.321848)),
    ('lForeArm', 7, (10.220657, 0.024782, 0.183730)),
    ('lHand', 8, (10.719483, 0.309727, 0.636712)),
    ('rCollar', 2, (-0.580691, 8.059053, 0.760605)),
    ('rShldr', 10, (-6.222462, 0.009832, -0.321848)),
    ('rForeArm', 11, (-10.220657, 0.024782, 0.183730)),
    ('rHand', 12, (-10.719483, 0.309727, 0.636712)),
    ('lThigh', 0, (4.361177, -6.202389, -1.775974)),
    ('lShin', 14, (-1.317052, -18.333157, 1.143369)),
    ('lFoot', 15, (-0.636132, -16.703447, -0.302338)),
    ('rThigh', 0, (-4.361177, -6.202389, -1.775974)),
    ('rShin', 17, (1.317052, -18.333157, 1.143369)),
    ('rFoot', 18, (0.636132, -16.703447, -0.302338)),
]
END_SITES = {'head': (0, 10.033302, 0), 'lHand': (10.033305, 0, 0), 'rHand': (-10.033305, 0, 0),
             'lFoot': (0, -2.207327, 5.819317), 'rFoot': (0, -2.207327, 5.819317)}
# standing hip height (hip -> thigh -> shin -> foot -> sole)
HIP_Y = 6.202389 + 18.333157 + 16.703447 + 2.207327


def hierarchy():
    out = ['HIERARCHY']
    kids = {i: [k for k, b in enumerate(BONES) if b[1] == i] for i in range(len(BONES))}

    def emit(i, depth):
        name, parent, off = BONES[i]
        pad = '\t' * depth
        out.append(pad + ('ROOT ' if parent < 0 else 'JOINT ') + name)
        out.append(pad + '{')
        out.append(pad + '\tOFFSET %.6f %.6f %.6f' % off)
        out.append(pad + ('\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation'
                          if parent < 0 else '\tCHANNELS 3 Zrotation Yrotation Xrotation'))
        for k in kids[i]:
            emit(k, depth + 1)
        if name in END_SITES:
            out.append(pad + '\tEnd Site')
            out.append(pad + '\t{')
            out.append(pad + '\t\tOFFSET %.6f %.6f %.6f' % END_SITES[name])
            out.append(pad + '\t}')
        out.append(pad + '}')
    emit(0, 0)
    return '\n'.join(out)


TRUNCATED = set()


def blocks(text):
    """(name, keys) for every animation block in text; keys: list of morph value lists.
    Some 2008 exports were cut off mid-frame, with the next animation's name written into
    the broken key line ("morph=3,0,0,0,walkinlookaroundBVH6200"): the broken key is
    dropped, the name starts its own block, and the cut animation is noted in TRUNCATED."""
    def fix(m):
        TRUNCATED.add('pending')
        return m.group(2)
    text = re.sub(r'^(morph=[-\d.,eE]*?,?)([A-Za-z_][\w.-]*BVH\d+)\s*$', fix, text, flags=re.M | re.I)
    L = text.split('\n')
    out, i = [], 0
    while i < len(L) - 2:
        if re.fullmatch(r'\d+', L[i + 1].strip()) and L[i + 2].startswith('0,'):
            name, j, keys = L[i].strip(), i + 2, []
            while j < len(L) and (re.match(r'^[\d.]+,[\d.]+\s*$', L[j]) or L[j].startswith('morph=')):
                if L[j].startswith('morph='):
                    try:
                        v = [float(x) for x in L[j][6:].strip().split(',')]
                    except ValueError:
                        v = []
                    if len(v) == 7:
                        keys.append(v)
                j += 1
            out.append((name, keys))
            i = j
        else:
            i += 1
    return out


def base(name):
    return re.sub(r'(\.?bvh)\d*$', '', name.strip(), flags=re.I)


def to_bvh(keys):
    nb = len(BONES)
    frames = [keys[k:k + nb] for k in range(0, len(keys) - nb + 1, nb)]
    lines = [hierarchy(), 'MOTION', 'Frames: %d' % len(frames), 'Frame Time: %.6f' % KEY_TIME]
    for fr in frames:
        vals = []
        for b, m in enumerate(fr):
            if int(m[0]) != b:
                raise ValueError('bone order broken at bone %d' % b)
            if b == 0:
                vals += [m[1] / SCALE, m[2] / SCALE + HIP_Y, m[3] / SCALE]
            vals += [m[6], m[5], m[4]]            # Z Y X
        lines.append(' '.join('%.5f' % v for v in vals))
    return '\n'.join(lines) + '\n', len(frames)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(INPUT, 'BVH', 'iGame3D'))
    ap.add_argument('--include-originals', action='store_true')
    a = ap.parse_args(argv)
    found = {}                                    # base name (lower) -> (name, keys, source)
    for p in sorted(glob.glob(os.path.join(INPUT, 'WIF', '*.wtf'))):
        t = open(p, errors='replace').read()
        for name, keys in blocks(t[t.find('#Animations'):]):
            found.setdefault(base(name).lower(), (base(name), keys, 'WIF/' + os.path.basename(p)))
    for p in sorted(glob.glob(os.path.join(INPUT, 'BVH', 'Conversions', '*', '*.txt'))):
        for name, keys in blocks(open(p, errors='replace').read()):
            found.setdefault(base(name).lower(), (base(name), keys, os.path.relpath(p, INPUT)))
    originals = set()
    for p in glob.glob(os.path.join(INPUT, 'BVH', '*', '*')):
        if p.lower().endswith('.bvh') and not p.startswith(a.out):
            originals.add(os.path.splitext(os.path.basename(p))[0].lower())
    os.makedirs(a.out, exist_ok=True)
    written, skipped, bad, cut = 0, [], [], []
    for key, (name, keys, src) in sorted(found.items()):
        if key in originals and not a.include_originals:
            skipped.append(name)
            continue
        if len(keys) < len(BONES) * 2:
            bad.append('%s (%s): only %d keys' % (name, src, len(keys)))
            continue
        if len(keys) % len(BONES):
            cut.append(name)
        try:
            text, n = to_bvh(keys)
        except ValueError as e:
            bad.append('%s (%s): %s' % (name, src, e))
            continue
        safe = re.sub(r'[^\w.-]+', '_', name)
        with open(os.path.join(a.out, safe + '.bvh'), 'w', newline='\n') as fh:
            fh.write(text)
        written += 1
    print('animations found: %d; BVH written: %d -> %s' % (len(found), written, a.out))
    print('skipped (original BVH already here): %d %s' % (len(skipped), ', '.join(skipped)))
    if cut:
        print('cut off in 2008 (last partial frame dropped): %d %s' % (len(cut), ', '.join(cut)))
    if bad:
        print('could not convert: %d' % len(bad))
        for b in bad:
            print('  ' + b)


if __name__ == '__main__':
    sys.exit(main())
