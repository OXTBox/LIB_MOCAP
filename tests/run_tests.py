#!/usr/bin/env python3
"""LIB_MOCAP's check that runs without LiveCode (A-50): a two-joint BVH made here is parsed, and
forward kinematics puts the child joint where its offset says.

    python3 -I tests/run_tests.py
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tools'))
import bvh                                    # noqa: E402

CHECKS, FAILS = [0], []


def check(name, ok, got=None):
    CHECKS[0] += 1
    if not ok:
        FAILS.append(name)
        print("FAIL  %s%s" % (name, "" if got is None else "  (got %r)" % (got,)))
    else:
        print("ok    %s" % name)


def done():
    print("\n%d of %d passed" % (CHECKS[0] - len(FAILS), CHECKS[0]))
    return 1 if FAILS else 0


BVH = '''HIERARCHY
ROOT Hips
{
  OFFSET 0 0 0
  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
  JOINT Spine
  {
    OFFSET 0 10 0
    CHANNELS 3 Zrotation Xrotation Yrotation
    End Site
    {
      OFFSET 0 5 0
    }
  }
}
MOTION
Frames: 2
Frame Time: 0.0333333
0 0 0 0 0 0 0 0 0
1 2 3 0 0 0 0 0 0
'''


def main():
    joints, motion, ftime = bvh.parse(BVH)
    names = [j['name'] for j in joints]
    check('three joints (root, spine, end site)', names == ['Hips', 'Spine', None], names)
    check('spine is a child of the root', joints[1]['parent'] == 0)
    check('two frames of nine channels', len(motion) == 2 and all(len(f) == 9 for f in motion))
    check('frame time', abs(ftime - 0.0333333) < 1e-6, ftime)
    pos = bvh.fk(joints, motion[1])
    close = lambda a, b: all(abs(x - y) < 1e-9 for x, y in zip(a, b))
    check('forward kinematics: root moved to 1,2,3, spine 10 above it, end site 15 above', close(pos[0], (1, 2, 3))
          and close(pos[1], (1, 12, 3)) and close(pos[2], (1, 17, 3)), pos)
    frame = list(motion[1]); frame[5] = 90         # turn the root 90 degrees about Y: still straight up
    check('a turn about Y keeps a vertical spine vertical', close(bvh.fk(joints, frame)[2], (1, 17, 3)))
    return done()


if __name__ == '__main__':
    sys.exit(main())
