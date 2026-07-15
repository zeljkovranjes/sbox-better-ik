namespace BetterIk.Maths;

using Quaternion = System.Numerics.Quaternion;

/// <summary>Per-foot output of <see cref="FootPlacementSolver.Solve"/>. Unweighted - the
/// aggregate <see cref="FootPlacementInput.Weight"/> is applied only to
/// <see cref="FootPlacementResult.PelvisOffset"/>, not here (see that type's doc comment).</summary>
public readonly struct FootResult
{
    /// <summary>True when a hit existed, its normal was usable, and the slope was within limit.</summary>
    public readonly bool Grounded;

    /// <summary>Clamped, unweighted, measured along the up axis. 0 when not grounded.</summary>
    public readonly float VerticalDelta;

    /// <summary>Ground-aligned foot rotation goal, unweighted. Equals the input FootRotation
    /// exactly when not grounded.</summary>
    public readonly Quaternion TargetRotation;

    public FootResult(bool grounded, float verticalDelta, Quaternion targetRotation)
    {
        Grounded = grounded;
        VerticalDelta = verticalDelta;
        TargetRotation = targetRotation;
    }
}
