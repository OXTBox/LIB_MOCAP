# LIB_MOCAP

Video → motion capture → animated characters in xt3d and Vibecode3D, all on local hardware.
Capture comes from MoCapAnything (`D:\AI\PixelArtistry-MoCapAnything-main`), which turns any
video into a skeleton animation.

| Path | What it is |
|---|---|
| `input/` | Drop captures here: `*_skeleton.glb` (+ the BVH beside it for joint names), one folder per capture |
| `library/<name>/` | Made by `mocap_lib.py process`: skinned GLB, full clip, stick clip, `score.json`, `NOTES.md` |
| `CATALOG.md` | Every clip with kind, length, travel speed, score and issues (generated) |
| `tools/mocap_lib.py` | `process` input → library, `procwalk` the placeholder walk, `catalog` |
| `tools/mocap_skin.py` | Skeleton GLB → skinned procedural body GLB (xt3d) + plain-text clip (xTalk) |

## Workflow

```
# 1. put a MoCapAnything result (folder or files) into input/
python3 -I tools/mocap_lib.py process      # 2. every capture -> library/<name>/, CATALOG.md
# 3. read CATALOG.md; write what you notice in library/<name>/NOTES.md (kept forever)
```

**Stick clip** (`<name>_stick.txt`, what Vibecode3D games use): the minimal look. 25 joints,
no fingers, lines only. In place (hips over 0,0), turned to face +Z every frame, feet on
y = 0, y up, metres. Header: `fps`, `frames`, `travel` (forward m/s the game should move the
figure so the feet don't skate), `names`, `parents`, `edges` (`a-b`, 0-based).

**Score** (0-100): minus jitter (mean joint acceleration above 4 m/s²), foot sliding while
planted, a gap between last and first pose (does not loop), and floating feet. Issues are
listed in words. Kind: `locomotion`, `turn` or `in place`.

**proc_walk**: a procedural placeholder walk on the same 25 joints, with the capture's
proportions. Feet are planned (60% stance, flat, constant speed back; 40% swing) and the
knees solved by two-bone IK, so it scores 100 (no slide, loops). Replace with a captured
walk as soon as there is one.

## What MoCapAnything gives us (input/Video_To_Mocap_Example_1)

| File | Contents |
|---|---|
| `*_skeleton.glb` | 65 joint nodes `j0..j64` (Mixamo order), 64 rigid "bone_N" stick meshes, one animation (65 rotations + hip translation). **No skin**, so engines cannot deform a mesh with it as is. 145 keys spread over 9.6 s (15 fps) |
| `*_rot6d_pred.bvh` | Same motion as BVH, real joint names (`mixamorig:Hips` ...), 145 frames at 30 fps (4.8 s) |
| `*_pose_pred.npy` | float32 `(145, 150, 3)`: joint positions per frame (150 slots; the rig uses 65) |
| `*_rot6d_pred.npy` | float32 `(145, 150, 6)`: joint rotations in 6D form |
| `_skel.mp4` | Preview render of the skeleton |

The GLB and BVH disagree on timing (15 vs 30 fps); the BVH's 30 fps matches the video.
The rig is small: 1.41 m to the top of the head, legs about 0.57 m. Example 1 scored 77:
no root motion, turns 335° in place, does not loop. Its feet do touch the floor (the
ankle joints sit 9-13 cm up, as ankles do). Used as the idle clip in Play.Heroes.

## mocap_skin.py

```
python3 -I tools/mocap_skin.py input/<name>/<id>_skeleton.glb --out somewhere/name
    [--fps 30]     capture rate used to retime the keys (0 = keep the GLB's times)
    [--thick 1.0]  body thickness
```

- **`<out>_skinned.glb`**: one mesh skinned to the 65 joints (real names from the BVH), the
  animation retimed to 30 fps. The body is an octagonal prism per bone plus a head; each
  bone's far end blends 50/50 into the next joint so elbows and knees bend smoothly. Vertex
  colours: left blue, right red, centre white. 1,746 vertices.
  Verified in the xt3d bench (2026-11-12): loads as 1 mesh / 65 bones / 289 frames (raylib
  resamples to 60 fps), deforms correctly through the clip.
  In xt3d: `xt3d_LoadModel`, `xt3d_LoadModelAnimations`, `xt3d_UpdateModelAnimation model, anim, frame`.
- **`<out>_clip.txt`**: for xTalk. Header lines `fps`, `frames`, `joints`, `names,...`,
  `parents,...` (-1 = root), then one line per frame: x,y,z of every joint in world space
  (metres, Y up). Draw a line from each joint to its parent for a vector skeleton, or place
  Vibecode3D shapes on the joints.

Standard library only (no numpy).
