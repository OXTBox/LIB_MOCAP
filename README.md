# LIB_MOCAP

Video → motion capture → animated characters in xt3d and Vibecode3D, all on local hardware.
Capture comes from MoCapAnything (`D:\AI\PixelArtistry-MoCapAnything-main`), which turns any
video into a skeleton animation.

| Path | What it is |
|---|---|
| `../LIB_LLMIN/LIB_MOCAP/` | Drop NEW captures here: MoCapAnything `*_skeleton.glb` (+ its BVH), or any `.bvh`. After processing they move to `../LIB_LLMOUT/Read/LIB_MOCAP/` |
| `../LIB_BVH/` | The BVH motion library (Animazoo, FreeBVH/Truebones, iGame3D conversions): its own local repo since 2026-11-12 |
| `input/` | Example_1 (in git) and older inputs (obj, wtf) |
| `library/<name>/` | Made by `mocap_lib.py process`: skinned GLB, full clip, stick clip, `score.json`, `NOTES.md`. Git keeps only `score.json` and `NOTES.md`, plus the motion files a game uses (see `.gitignore`); rebuild the rest with `process` (about 5 minutes) |
| `CATALOG.md` | Every clip with kind, length, travel speed, score and issues (generated) |
| `tools/mocap_lib.py` | `process` input → library, `procwalk` the placeholder walk, `catalog` |
| `tools/mocap_skin.py` | Skeleton GLB → skinned procedural body GLB (xt3d) + plain-text clip (xTalk) |
| `tools/wif.py` | iGame3D animation tables (`input/WIF/*.wtf`, `../LIB_BVH/Conversions/*.BVH.txt`) → real BVH files in `../LIB_BVH/iGame3D/` |
| `tools/bvh.py` | BVH reader: forward kinematics in each file's own rotation order, joint names mapped to Mixamo-style names (Animazoo, Truebones/Poser, CMU, Mixamo) |

## Workflow

```
# 1. put a MoCapAnything result (folder or files) into ../LIB_LLMIN/LIB_MOCAP/
python3 -I tools/mocap_lib.py process      # 2. every capture -> library/<name>/, CATALOG.md
python3 -I tools/mocap_lib.py process --new   #    only what is not in the library yet
python3 -I tools/mocap_lib.py glb NAME        #    skinned body GLB for xt3d from a BVH clip (on demand)
# 3. read CATALOG.md; write what you notice in library/<name>/NOTES.md (kept forever)
```

**Stick clip** (`<name>_stick.txt`, what Vibecode3D games use): the minimal look. Up to 27
joints (no fingers), lines only, at most 30 fps. In place (hips over 0,0), turned to face +Z
every frame (the toes settle which way is forward, so mirrored rigs come out right), feet on
y = 0, y up, metres. **Every clip is scaled to the same body** (hip to ankle 0.85 m, about
1.75 m tall), whatever units the file used. Header: `fps`, `frames`, `travel` (forward m/s
the game should move the figure so the feet don't skate), `names`, `parents`, `edges`
(`a-b`, 0-based), and for walks and runs `cycle,a,b` (1-based frames that loop cleanly) and
`cycle_travel` (the speed during that cycle). Games should loop the cycle.

**Score** (0-100): minus jitter, foot sliding while planted, a gap between last and first
pose (does not loop), and floating feet. Issues are listed in words. Kind: `locomotion`
(with its loop cycle), `turn` or `in place`.
**Jitter** is a noise index that ignores speed, scale and frame rate: for smooth motion the
wobble about the neighbours' midpoint one frame away is 1/4 of that two frames away; for
noise they are equal. 0% smooth, 100% noise; over 45% is flagged. Typical: 120 fps captures
~0%, Animazoo 60 fps 20-45%, near-still poses higher (sensor noise shows when nothing moves).

**Sources (2026-11-12; the BVH collections are in ../LIB_BVH)**: `BVH/Animazoo` (41 clips) and `BVH/FreeBVH` (Truebones
free pack, 21; a horse and a hands-only clip are skipped) are processed. `BVH/iGame3D`
(938 clips) was rebuilt from iGame3D's 2008 tables by `tools/wif.py` (next section); after
that `WIF/` and `BVH/Conversions/` are no longer needed. **Rights**: per the owner (2026-11-12),
Truebones' clips are repackaged from the **CMU Graphics Lab Motion Capture Database**
(mocap.cs.cmu.edu), so `FreeBVH/` and the 938 `iGame3D/` clips are really CMU motion. CMU's
data is free to use, commercial games included; credit it ("The data used in this project
was obtained from mocap.cs.cmu.edu. The database was created with funding from NSF
EIA-0196217.") and do not resell the raw data as a motion library. Check the current terms
on mocap.cs.cmu.edu before shipping. `Animazoo/` has no stated licence: local use only
until settled.
**Possible upgrade**: CMU's originals (120 fps, true timing) are still published; matching
our clips to them would replace the estimated 0.2 s-a-key timing of the iGame3D clips.

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
ankle joints sit 9-13 cm up, as ankles do). Used as the idle clip in Play.Amazing_Couple (LIB_GAMES).

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

## iGame3D tables → BVH (tools/wif.py)

In 2008 iGame3D turned the Truebones BVH library (20-bone Poser-style rig) into its own
tables, stored twice: `input/WIF/*.wtf` (models with `#Animations`, 940 motions) and
`../LIB_BVH/Conversions/*.BVH.txt` (one per file, 848; 830 are the same as in WIF, 35 files
are empty stubs). A key per bone: `morph=bone,tx,ty,tz,rx,ry,rz`.

- **Rotation convention**, fitted against the 16 tables whose original Truebones BVH is in
  `FreeBVH/`: R = Rz · Ry · Rx, all angles as stored. Rebuilt poses match the originals to
  1-2% of body height; in-between frames (after slerp to 30 fps) to 0.5%.
- **Position**: root only, BVH units × 0.005263 (undone).
- **Timing is not stored.** Most tables kept every 6th frame of a 30 fps original (0.2 s a
  key), some every 6th of 60 fps or every 3rd of 30 fps (0.1 s). `wif.py` uses 0.2 s, right
  for about 2 in 3; the catalog marks these clips "timing estimated".
- 958 unique motions: 938 written, 16 skipped (the original BVH is better), 5 were cut off
  mid-frame in 2008 (aerobdance, cheer, coach2, rebound, walkandwonder: last partial frame
  dropped), 1 holds a single frame (loadpistol).
- `bvh.py` slerps any file slower than 24 fps up to 30 fps when reading, so the stick clips
  of these 5-key-a-second files are smooth.

**Checked against iGame3D itself (2026-11-12).** `tools/ig3d_bones.lua`, run by
`igame3dvs.exe` in `~/projects/iGame3D_2026_CLAUDE`, plays WIF animations in the engine
and logs every bone's world position per frame; poses match `wif.py`'s BVH frames to
0.2-2.5% of leg length, including `bees`, which exists only as an iGame3D table. The
engine plays these tables at about 0.1 s a key (twice the originals' speed for most);
`wif.py` keeps 0.2 s, which matches the surviving originals.
`input/WIF` was removed after this check (a copy stays in Program Files);
`../LIB_BVH/Conversions` was removed too; its 18 tables not in WIF live on only as BVH in `../LIB_BVH/iGame3D`.

## Licence

GPL-3.0 (see `LICENSE`), because the xTalk work here is built on the GPL-licensed LiveCode Community 9 engine. Copyright (c) 2026 xAction.

The Python files (`.py`) and other material that is not xTalk are also offered under the MIT licence, Copyright (c) 2026 xAction: you may use them under either licence.
