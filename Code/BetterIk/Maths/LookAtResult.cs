namespace BetterIk.Maths;

using Quaternion = System.Numerics.Quaternion;

/// <summary>Output of <see cref="LookAtSolver.Solve"/>.</summary>
public struct LookAtResult
{
    /// <summary>Final blended world rotation to assign to the bone.</summary>
    public Quaternion BoneRotation;

    /// <summary>False only when the target coincides with the bone position (no direction to aim at).</summary>
    public bool Solved;
}
