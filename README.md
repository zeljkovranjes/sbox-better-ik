# Better IK

Runtime IK for s&box. Drop a component on the end bone of any rigged model (hand, foot, head, tail) and it works: no per-model setup files, no bone-name markup, no assumed axis convention.

- **TwoBoneIK** - arm/leg IK with pole vector control.
- **LookAtIK** - single-bone look-at (head/eye aim), clamped to a maximum angle.
- **FootPlacementIK** - ground conformance (foot planting + pelvis drop) on top of TwoBoneIK.
- **FabrikIK** - unconstrained variable-length chain IK (tails, tentacles, ropes).

See the [wiki](https://github.com/zeljkovranjes/sbox-better-ik/wiki) for setup and usage.
