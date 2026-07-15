namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;

/// <summary>Precomputed once per bone from the bind pose. Result of <see cref="LookAtSolver.AnalyzeBindPose"/>.</summary>
public readonly struct LookAtBindData
{
    /// <summary>Unit, bone-local space direction the bone should be treated as "facing".</summary>
    public readonly Vector3 LocalAimDirection;

    /// <summary>False if no usable child bone existed in the bind pose (leaf bone, or a degenerate
    /// child offset); <see cref="LocalAimDirection"/> is still a valid unit vector, just an arbitrary
    /// deterministic fallback rather than an anatomically meaningful one.</summary>
    public readonly bool IsReliable;

    public LookAtBindData(Vector3 localAimDirection, bool isReliable)
    {
        LocalAimDirection = localAimDirection;
        IsReliable = isReliable;
    }
}
