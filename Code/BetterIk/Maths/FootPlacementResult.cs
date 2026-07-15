namespace BetterIk.Maths;

/// <summary>Output of <see cref="FootPlacementSolver.Solve"/>.</summary>
public struct FootPlacementResult
{
    public FootResult LeftFoot;
    public FootResult RightFoot;

    /// <summary>Clamped AND weight-scaled, measured along the up axis. Final, write-ready -
    /// unlike the per-foot results, this already has <see cref="FootPlacementInput.Weight"/>
    /// applied (see that type's doc comment for why the split is asymmetric).</summary>
    public float PelvisOffset;

    /// <summary>False only when <see cref="FootPlacementInput.UpAxis"/> is degenerate.</summary>
    public bool Solved;
}
