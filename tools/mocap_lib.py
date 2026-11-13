#!/usr/bin/env python3
"""mocap_lib.py - the LIB_MOCAP library: process captures, score them, keep a catalog.

    python3 -I tools/mocap_lib.py process [NAME ...]   every capture in input/ (or just NAME)
    python3 -I tools/mocap_lib.py procwalk             the procedural placeholder walk
    python3 -I tools/mocap_lib.py catalog              rebuild CATALOG.md from library/

A capture is a *_skeleton.glb in input/ (a BVH beside it supplies the joint names). Its
name is the subfolder it sits in (input/<name>/...), or the file name without
"_skeleton". Each one becomes library/<name>/:

    <name>_skinned.glb   procedural body skinned to the skeleton (xt3d)       [mocap_skin.py]
    <name>_clip.txt      65 joint world positions per frame (xTalk)          [mocap_skin.py]
    <name>_stick.txt     the minimal stick figure for Vibecode3D (below)
    score.json           the measurements and issues
    NOTES.md             human notes; created once, never overwritten

Stick clip (plain text, y up, metres): 25 joints (no fingers), edges, one line per frame.
Made in place (hips over 0,0), turned to face +Z every frame, and lowered so the lowest
foot point of the whole clip touches y = 0. The removed root motion is kept as
`travel` (metres a second, forward) so a game can match its walking speed to the feet.

Score (0-100, see `score()`): starts at 100, minus jitter, foot sliding and a loop gap.
Floating feet and missing root motion are reported but cost little: the stick fixes the
first, and in-place clips are what games drive anyway.
Standard library only.
"""
import json, math, os, re, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
INPUT = os.path.join(ROOT, 'input')
LIBRARY = os.path.join(ROOT, 'library')
sys.path.insert(0, HERE)
import mocap_skin  # noqa: E402

STICK = ('Hips Spine Spine1 Spine2 Neck Head HeadTop_End '
         'LeftShoulder LeftArm LeftForeArm LeftHand RightShoulder RightArm RightForeArm RightHand '
         'LeftUpLeg LeftLeg LeftFoot LeftToeBase LeftToe_End '
         'RightUpLeg RightLeg RightFoot RightToeBase RightToe_End').split()
FEET = ('LeftFoot', 'LeftToeBase', 'LeftToe_End', 'RightFoot', 'RightToeBase', 'RightToe_End')


# ---------------------------------------------------------------------------- clip io
def read_clip(path):
    head, frames = {}, []
    for line in open(path, encoding='utf-8').read().splitlines():
        if not line or line.startswith('#'):
            continue
        if re.match(r'^-?[\d.]', line):
            v = [float(x) for x in line.split(',')]
            frames.append([tuple(v[i:i + 3]) for i in range(0, len(v), 3)])
        else:
            k, _, rest = line.partition(',')
            head[k] = rest
    head['names'] = head['names'].split(',')
    head['parents'] = [int(p) for p in head['parents'].split(',')]
    return head, frames


def subset(head, frames, keep=STICK):
    names, parents = head['names'], head['parents']
    idx = [names.index(n) for n in keep if n in names]
    pos = {j: k for k, j in enumerate(idx)}
    par = []
    for j in idx:
        p = parents[j]
        while p >= 0 and p not in pos:
            p = parents[p]
        par.append(pos.get(p, -1))
    return [names[j] for j in idx], par, [[f[j] for j in idx] for f in frames]


# ---------------------------------------------------------------------------- measuring
def facing(f, names):
    """forward yaw (radians, 0 = +Z) from the hip line"""
    l, r = f[names.index('LeftUpLeg')], f[names.index('RightUpLeg')]
    rx, rz = r[0] - l[0], r[2] - l[2]
    return math.atan2(rz, -rx)          # forward = up x right = (rz, -rx); yaw = atan2(fx, fz)


def score(names, frames, fps, inplace_speed=0.0):
    """inplace_speed: for in-place clips, the forward speed the game adds (planted feet
    should move backward at exactly that speed)"""
    n = len(frames)
    dt = 1.0 / fps
    hips = [f[names.index('Hips')] for f in frames]
    travel_path = sum(math.dist((a[0], a[2]), (b[0], b[2])) for a, b in zip(hips, hips[1:]))
    net = math.dist((hips[0][0], hips[0][2]), (hips[-1][0], hips[-1][2]))
    yaws = [facing(f, names) for f in frames]
    turn = 0.0
    for a, b in zip(yaws, yaws[1:]):
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        turn += d
    feet = [names.index(x) for x in FEET if x in names]
    floor = min(f[j][1] for f in frames for j in feet)
    # jitter: mean size of each joint's acceleration, m/s^2
    acc, cnt = 0.0, 0
    for a, b, c in zip(frames, frames[1:], frames[2:]):
        for j in range(len(names)):
            acc += math.dist(a[j], (2 * b[j][0] - c[j][0], 2 * b[j][1] - c[j][1], 2 * b[j][2] - c[j][2])) / dt ** 2
            cnt += 1
    jitter = acc / max(1, cnt)
    # foot slide: a foot point within 2 cm of the floor that still moves sideways
    planted, sliding = 0, 0
    for a, b in zip(frames, frames[1:]):
        for j in feet:
            if a[j][1] - floor < 0.02 and b[j][1] - floor < 0.02:
                planted += 1
                vx, vz = (b[j][0] - a[j][0]) / dt, (b[j][2] - a[j][2]) / dt + inplace_speed
                if math.hypot(vx, vz) > 0.25:
                    sliding += 1
    slide = sliding / planted if planted else 0.0
    # loop gap: first vs last pose, in place and facing +Z
    st = stick_frames(names, [frames[0], frames[-1]], ground=0)
    gap = sum(math.dist(p, q) for p, q in zip(st[0], st[1])) / len(names)
    s = 100.0
    s -= min(40, max(0, jitter - 4) * 2)          # calm captures sit around 2-4 m/s^2
    s -= min(30, slide * 60)
    s -= min(20, max(0, gap - 0.03) * 150)
    s -= 5 if floor > 0.03 else 0
    issues = []
    if travel_path < 0.05 * n * dt + 0.05:
        issues.append('no root motion (hips locked: plays in place)')
    if abs(math.degrees(turn)) > 45:
        issues.append('turns %d degrees in place' % round(math.degrees(turn)))
    if floor > 0.03:
        issues.append('feet float %d cm above the floor (stick is grounded)' % round(floor * 100))
    if floor < -0.03:
        issues.append('feet go %d cm below the floor' % round(-floor * 100))
    if jitter > 8:
        issues.append('jittery (%.1f m/s^2 mean joint acceleration)' % jitter)
    if slide > 0.2:
        issues.append('feet slide on %d%% of planted frames' % round(slide * 100))
    if gap > 0.08:
        issues.append('does not loop (%d cm mean gap between last and first pose)' % round(gap * 100))
    speed = net / (n * dt)
    kind = 'locomotion' if speed > 0.3 else ('turn' if abs(math.degrees(turn)) > 90 else 'in place')
    return {'frames': n, 'fps': fps, 'seconds': round(n * dt, 2), 'kind': kind,
            'root_path_m': round(travel_path, 3), 'travel_m_per_s': round(speed, 3),
            'turn_deg': round(math.degrees(turn)), 'floor_m': round(floor, 3),
            'jitter_m_per_s2': round(jitter, 2), 'foot_slide': round(slide, 3), 'loop_gap_m': round(gap, 3),
            'score': max(0, round(s)), 'issues': issues}


def stick_frames(names, frames, ground=None):
    """in place, facing +Z, lowest foot point at y = 0 (ground: that offset, or None = measure)"""
    hi = names.index('Hips')
    out = []
    for f in frames:
        yaw = facing(f, names)
        c, s = math.cos(-yaw), math.sin(-yaw)
        hx, hz = f[hi][0], f[hi][2]
        out.append([((p[0] - hx) * c + (p[2] - hz) * s, p[1], -(p[0] - hx) * s + (p[2] - hz) * c) for p in f])
    if ground is None:
        feet = [names.index(x) for x in FEET if x in names]
        ground = min(f[j][1] for f in out for j in feet)
    return [[(p[0], p[1] - ground, p[2]) for p in f] for f in out]


def write_stick(path, name, names, parents, frames, fps, travel, note):
    edges = ' '.join('%d-%d' % (p, k) for k, p in enumerate(parents) if p >= 0)
    lines = ['# LIB_MOCAP stick clip: %s' % note,
             'name,%s' % name, 'fps,%g' % fps, 'frames,%d' % len(frames), 'travel,%g' % travel,
             'joints,%d' % len(names), 'names,' + ','.join(names),
             'parents,' + ','.join(str(p) for p in parents), 'edges,' + edges,
             '# frame lines: x,y,z per joint (metres, y up, facing +Z, in place, feet on y = 0)']
    for f in frames:
        lines.append(','.join('%.3f' % c for p in f for c in p))
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')


# ---------------------------------------------------------------------------- process
def captures():
    found = []
    if not os.path.isdir(INPUT):
        return found
    for dirpath, _, files in os.walk(INPUT):
        for f in sorted(files):
            if f.lower().endswith('.glb'):
                rel = os.path.relpath(dirpath, INPUT)
                name = rel.split(os.sep)[0] if rel != '.' else re.sub(r'_skeleton$', '', f[:-4])
                found.append((re.sub(r'[^\w.-]+', '_', name), os.path.join(dirpath, f)))
    return found


def process(name, glb):
    out = os.path.join(LIBRARY, name)
    os.makedirs(out, exist_ok=True)
    base = os.path.join(out, name)
    info = mocap_skin.convert(glb, base, 30, 1.0)
    head, frames = read_clip(base + '_clip.txt')
    names, parents, sub = subset(head, frames)
    sc = score(names, sub, float(head['fps']))
    sc.update({'name': name, 'source': os.path.relpath(glb, ROOT), 'joints_names_from': info['names_from'],
               'glb_key_rate': round(info['glb_rate'], 2), 'procedural': False})
    st = stick_frames(names, sub)
    write_stick(base + '_stick.txt', name, names, parents, st, float(head['fps']), sc['travel_m_per_s'],
                'from %s' % os.path.basename(glb))
    with open(os.path.join(out, 'score.json'), 'w') as fh:
        json.dump(sc, fh, indent=1)
    notes = os.path.join(out, 'NOTES.md')
    if not os.path.exists(notes):
        with open(notes, 'w') as fh:
            fh.write('# %s\n\n(one line about the motion, then anything found while using it)\n' % name)
    return sc


def procwalk():
    """a placeholder walk cycle on the stick skeleton, proportions from the first capture"""
    name = 'proc_walk'
    out = os.path.join(LIBRARY, name)
    os.makedirs(out, exist_ok=True)
    ref = None
    for d in sorted(os.listdir(LIBRARY)):
        p = os.path.join(LIBRARY, d, d + '_stick.txt')
        if d != name and os.path.exists(p):
            ref = p
            break
    if ref:
        head, frames = read_clip(ref)
        names, f0 = head['names'], frames[0]
    else:
        sys.exit('procwalk needs one processed capture for body proportions')
    P = {n: f0[k] for k, n in enumerate(names)}
    L = lambda a, b: math.dist(P[a], P[b])
    thigh, shin = L('LeftUpLeg', 'LeftLeg'), L('LeftLeg', 'LeftFoot')
    foot, toe = L('LeftFoot', 'LeftToeBase'), L('LeftToeBase', 'LeftToe_End')
    upper, fore = L('LeftArm', 'LeftForeArm'), L('LeftForeArm', 'LeftHand')
    hipw = abs(P['LeftUpLeg'][0] - P['RightUpLeg'][0]) / 2
    shw = abs(P['LeftArm'][0] - P['RightArm'][0]) / 2
    up = {n: (P[n][0] - P['Hips'][0], P[n][1] - P['Hips'][1]) for n in names}
    fps, n = 30, 32
    STANCE = 0.6                       # stance share of the cycle
    STEP = 0.7 * (thigh + shin)        # step length: a normal walk for this leg length
    ANKLE = 0.075                      # ankle height with the foot flat
    reach = (thigh + shin) * 0.97
    LEGROOT = -up['LeftUpLeg'][1]       # the leg roots hang this far below the hips joint
    HIP = ANKLE + math.sqrt(reach * reach - (STEP / 2) ** 2) + LEGROOT
    frames_out = []
    for k in range(n):
        ph = 2 * math.pi * k / n
        hy = HIP - 0.015 * math.cos(4 * math.pi * k / n)     # a little lower at mid-stride
        pos = {}
        pos['Hips'] = (0, hy, 0)
        lean = 0.04
        for j in ('Spine', 'Spine1', 'Spine2', 'Neck', 'Head', 'HeadTop_End'):
            dy = up[j][1]
            pos[j] = (0, hy + dy, lean * dy)
        for side, sx, phase in (('Left', 1, 0), ('Right', -1, math.pi)):
            a = (ph + phase) % (2 * math.pi)
            t = a / (2 * math.pi)                            # 0..1 through this leg's cycle
            if t < STANCE:                                   # planted: straight back, steady speed
                u = t / STANCE
                fz, lift = STEP / 2 - STEP * u, 0.0
            else:                                            # swing: lift and come forward
                u = (t - STANCE) / (1 - STANCE)
                e = u * u * (3 - 2 * u)
                fz, lift = -STEP / 2 + STEP * e, 0.12 * math.sin(math.pi * u)
            hp = (sx * hipw, hy - LEGROOT, 0.0)
            an = (sx * hipw, ANKLE + lift, fz)
            kn = knee_ik(hp, an, thigh, shin)
            pitch = 0.5 * math.sin(math.pi * u) if t >= STANCE else 0.0   # toes droop in the swing
            tb = (an[0], an[1] - foot * 0.55 * math.cos(pitch) - 0.0, an[2] + foot * 0.83 * math.cos(pitch))
            te = (tb[0], tb[1] - toe * math.sin(pitch) * 0.5, tb[2] + toe * math.cos(pitch))
            pos[side + 'UpLeg'], pos[side + 'Leg'], pos[side + 'Foot'] = hp, kn, an
            pos[side + 'ToeBase'], pos[side + 'Toe_End'] = tb, te
            a = ph + phase
            arm = -0.35 * math.sin(a)                        # opposite to the leg
            shp = (sx * shw, hy + up[side + 'Arm'][1], lean * up[side + 'Arm'][1])
            pos[side + 'Shoulder'] = (sx * shw * 0.4, hy + up[side + 'Shoulder'][1], shp[2])
            el = (shp[0] + sx * 0.03, shp[1] - upper * math.cos(arm), shp[2] + upper * math.sin(arm))
            fa = arm - 0.35
            ha = (el[0], el[1] - fore * math.cos(fa), el[2] + fore * math.sin(fa) + 0.0)
            pos[side + 'Arm'], pos[side + 'ForeArm'], pos[side + 'Hand'] = shp, el, ha
        frames_out.append([pos[nm] for nm in names])
    ground = min(f[names.index(x)][1] for f in frames_out for x in FEET)
    frames_out = [[(p[0], p[1] - ground, p[2]) for p in f] for f in frames_out]
    # walking speed = how far an ankle travels back during stance (half a cycle), so the
    # game can move the figure at exactly that speed and the planted foot stays put
    speed = STEP / (STANCE * n / fps)
    base = os.path.join(out, name)
    write_stick(base + '_stick.txt', name, names, head['parents'], frames_out, fps, round(speed, 3),
                'PROCEDURAL placeholder walk (no capture), proportions from %s' % os.path.basename(ref))
    sc = score(names, frames_out, fps, speed)
    sc.update({'name': name, 'source': 'tools/mocap_lib.py procwalk', 'procedural': True,
               'travel_m_per_s': round(speed, 3), 'kind': 'locomotion (procedural, in place)'})
    sc['issues'] = ['procedural placeholder: replace with a captured walk'] + \
        [i for i in sc['issues'] if not i.startswith('no root motion')]
    with open(os.path.join(out, 'score.json'), 'w') as fh:
        json.dump(sc, fh, indent=1)
    return sc


def knee_ik(hip, ankle, l1, l2):
    """knee position in the leg's forward plane (x fixed), bending forward"""
    dz, dy = ankle[2] - hip[2], ankle[1] - hip[1]
    d = min(math.hypot(dz, dy), (l1 + l2) * 0.999)
    alpha = math.acos(max(-1.0, min(1.0, (l1 * l1 + d * d - l2 * l2) / (2 * l1 * d))))
    phi = math.atan2(dz, -dy)                       # angle of hip->ankle from straight down, forward +
    th = phi + alpha
    return (hip[0], hip[1] - l1 * math.cos(th), hip[2] + l1 * math.sin(th))


def catalog():
    rows = []
    for d in sorted(os.listdir(LIBRARY)) if os.path.isdir(LIBRARY) else []:
        p = os.path.join(LIBRARY, d, 'score.json')
        if not os.path.exists(p):
            continue
        sc = json.load(open(p))
        note = ''
        np_ = os.path.join(LIBRARY, d, 'NOTES.md')
        if os.path.exists(np_):
            lines = [l for l in open(np_).read().splitlines() if l.strip() and not l.startswith('#')
                     and not l.startswith('(one line')]
            note = lines[0] if lines else ''
        rows.append('| %s | %s | %s s | %s | **%s** | %s | %s |' % (
            d, sc['kind'], sc['seconds'], sc.get('travel_m_per_s', 0), sc['score'],
            '; '.join(sc['issues']) or '-', note))
    text = ['# LIB_MOCAP catalog (generated by tools/mocap_lib.py catalog)', '',
            'Score 0-100: jitter, foot sliding and a loop gap cost points (see tools/mocap_lib.py).',
            'Notes column: first line of library/<name>/NOTES.md (written by hand, never overwritten).', '',
            '| Name | Kind | Length | Travel m/s | Score | Issues | Notes |', '|---|---|---|---|---|---|---|'] + rows
    with open(os.path.join(ROOT, 'CATALOG.md'), 'w') as fh:
        fh.write('\n'.join(text) + '\n')
    return len(rows)


def main(argv=None):
    a = sys.argv[1:] if argv is None else argv
    if not a or a[0] not in ('process', 'procwalk', 'catalog'):
        print(__doc__)
        return 2
    if a[0] == 'process':
        todo = [(n, g) for n, g in captures() if len(a) == 1 or n in a[1:]]
        if not todo:
            print('nothing in %s' % INPUT)
        for n, g in todo:
            sc = process(n, g)
            print('%-32s score %3d  %s  %s' % (n, sc['score'], sc['kind'], '; '.join(sc['issues']) or 'no issues'))
    elif a[0] == 'procwalk':
        sc = procwalk()
        print('proc_walk score %d: %s' % (sc['score'], '; '.join(sc['issues'])))
    print('catalog: %d clips -> %s' % (catalog(), os.path.join(ROOT, 'CATALOG.md')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
