namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>
/// Per-frame input to <see cref="TwoBoneIkSolver.Solve"/>. All positions and rotations are in
/// one common space (the caller's choice, typically world space). Angles are radians.
/// </summary>
public struct TwoBoneIkInput
{
    public Vector3 RootPosition;
    public Vector3 MidPosition;
    public Vector3 EndPosition;
    public Quaternion RootRotation;
    public Quaternion MidRotation;
    public Quaternion EndRotation;

    public Vector3 TargetPosition;
    public Quaternion TargetRotation;

    /// <summary>True if a pole target is supplied via <see cref="PoleHint"/>.</summary>
    public bool HasPole;

    /// <summary>
    /// Direction from the root toward the desired elbow side (e.g. polePosition - RootPosition,
    /// or a rotated default direction). Only the component perpendicular to root-to-target is used;
    /// magnitude is ignored.
    /// </summary>
    public Vector3 PoleHint;

    /// <summary>Extra rotation applied about the root-to-target axis, after pole alignment.</summary>
    public float PoleAngleOffsetRadians;

    /// <summary>Bend-plane normal from <see cref="TwoBoneIkSolver.AnalyzeBindPose"/>, used as a
    /// fallback when the animated chain is collinear and has no usable bend plane of its own.</summary>
    public Vector3 FallbackBendNormal;

    /// <summary>0-1, clamped by the solver. Multiplied by <see cref="MasterWeight"/>.</summary>
    public float PositionWeight;

    /// <summary>0-1, clamped by the solver. Multiplied by <see cref="MasterWeight"/>.</summary>
    public float RotationWeight;

    /// <summary>0-1, clamped by the solver. Master blend for the whole IK result.</summary>
    public float MasterWeight;

    /// <summary>0 disables soft clamping (hard clamp at max reach). Sensible range [0, 0.5).</summary>
    public float SoftFraction;

    /// <summary>0 disables stretch. E.g. 0.2 allows the chain to lengthen up to 20% past max reach.</summary>
    public float MaxStretch;
}
