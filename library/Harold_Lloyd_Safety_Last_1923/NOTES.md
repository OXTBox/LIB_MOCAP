# Harold_Lloyd_Safety_Last_1923

Harold Lloyd dangling from the clock in "Safety Last!" (1923, public domain), 222 frames at 30 fps,
hips locked (plays in place), hands overhead nearly the whole take. Motion data only (from
LIB_BVH/Harold_Lloyd, the owner's capture); no film frames are kept here.

Used by:
- Vector Toss: `hang` 134-198 (Animations/VectorToss.anims.txt).
- DeadDays: cut into three moves for the temple step (Animations/DeadDays.anims.txt, 19 Nov 2026):
  - `climb` 1-44: hands rise from head height (1.7 m) to overhead (2.15 m) by frame 17, the feet
    swing back under him and stay low: reaching up and taking hold of the edge.
  - `hang` 132-182: arms straight (hands 2.05-2.2 m), legs low and kicking: the dangle. 132 is close
    to climb's last frame, 182 close to recover's first; it loops 182 -> 132 with a 13 cm mean gap.
  - `recover` 45-108: knees drawn up to 1.5 m, arms bend (hands 2.14 -> 1.83 m), head ducks
    (1.71 -> 1.44 m): hauling himself up. Follows climb with no jump (3 cm).
  - Picture of the cuts: cuts_climb_hang_recover.svg (8 frames of each, side view).
- The stick file is kept in git since 19 Nov (two games use it; the BVH stays local).
