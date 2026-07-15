namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>Per-frame input to <see cref="LookAtSolver.Solve"/>. Positions and rotations are in one
/// common space (typically world space). Angles are radians.</summary>
public struct LookAtInput
{
    public Vector3 BonePosition;

    /// <summary>Current animated world rotation, before this component's own adjustment.</summary>
    public Quaternion BoneRotation;

    /// <summary>Unit, bone-local space. From <see cref="LookAtBindData.LocalAimDirection"/>, cached
    /// once and not recomputed per frame.</summary>
    public Vector3 LocalAimDirection;

    public Vector3 TargetPosition;

    /// <summary>Clamp measured from the current animated aim direction each frame, not a bind-pose
    /// or world-space reference.</summary>
    public float MaxAngleRadians;

    /// <summary>0-1, clamped by the solver.</summary>
    public float Weight;
}
