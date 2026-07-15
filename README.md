# Better IK

Runtime IK for s&box. Drop a component on the end bone of any rigged model (hand, foot, head, tail) and it works: no per-model setup files, no bone-name markup, no assumed axis convention.

- **TwoBoneIK** - arm/leg IK with pole vector control.
- **LookAtIK** - single-bone look-at (head/eye aim), clamped to a maximum angle.
- **FootPlacementIK** - ground conformance (foot planting + pelvis drop) on top of TwoBoneIK.
- **FabrikIK** - unconstrained variable-length chain IK (tails, tentacles, ropes).

## Usage

### TwoBoneIK

Add to the model root (or any descendant - it finds the `SkinnedModelRenderer` automatically). Set **End Bone** to the hand/foot bone name and drag a **Target** GameObject; the root and mid bones are found automatically by walking up the skeleton. Drag a **Pole Target** to control which way the elbow/knee bends - if left empty, a sensible default is derived from the model's own bind pose (elbow back, knee forward). **Weight** blends the whole effect in/out at runtime (0 = untouched animation, 1 = fully solved); **Position Weight**/**Rotation Weight** split position and rotation separately. Advanced: manual root/mid bone overrides, a pole angle offset, and soft-reach/stretch for a softer max-reach limit instead of a hard clamp.

Gizmo: red text means the bone chain couldn't be resolved (check End Bone). The solved chain draws green when comfortably in reach, yellow when near max reach (only if soft reach is enabled), red when clamped or in the near-side dead zone.

### LookAtIK

Add anywhere on the model, set **Bone Name** to the head/eye bone and drag a **Target**. **Max Angle Degrees** clamps how far the bone can turn from its currently-animated facing direction - this lets the look-at cone naturally follow body/animation rotation instead of fighting it. **Weight** blends in/out. The bone's own local "forward" is derived from its bind-pose child bone; if it has no child (a leaf bone), a deterministic fallback axis is used and the gizmo's aim arrow turns orange to flag it - set **Local Aim Axis Override** to fix that manually.

Gizmo: cyan aim arrow with a cone at the clamp angle; orange means the auto-derived aim axis is a fallback guess, not read from bind-pose geometry.

### FootPlacementIK

Add to the model root, set **Left Foot Bone**/**Right Foot Bone** to the ankle/foot bone names. It traces down under each foot, plants them on real geometry, and drops the pelvis (auto-found as the nearest common ancestor of both legs) when the ground is lower so the far foot can still reach. **Weight** blends the whole effect. Ground group: **Max Step Up**/**Max Step Down** bound how far a foot will move to meet uneven terrain, **Max Pelvis Drop**/**Raise** bound the pelvis shift, **Max Ground Slope Degrees** ignores walls/steep faces, **Smoothing Rate** controls how quickly foot/pelvis offsets ease in rather than snapping. **Ignore Tags** (space-separated) excludes the character's own colliders from the ground trace.

Gizmo: green/red trace lines per foot (hit/miss), a magenta arrow showing the current pelvis shift, orange text if no pelvis bone could be auto-derived (use **Pelvis Bone Override**).

### FabrikIK

For chains TwoBoneIK can't handle - tails, tentacles, ropes, any number of bones. Unlike TwoBoneIK, it can't guess its own depth, so both **Root Bone** and **End Bone** must be set explicitly (walking up from End Bone until Root Bone is found by name). Drag a **Target**; **Weight** blends in/out; **Max Iterations**/**Tolerance** control the solve (Tolerance defaults to 0, meaning "auto" - proportional to the chain's own length). No pole vector or joint limits in this version - it's for floppy chains, not hinge-like limbs.

Gizmo: a yellow polyline along the chain plus a target marker; red text if the chain couldn't be resolved.
