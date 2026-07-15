namespace BetterIk.Maths;

/// <summary>Output of <see cref="FabrikSolver.Solve"/>.</summary>
public struct FabrikResult
{
    /// <summary>Solved joint positions, world space, same length and root-first ordering as the input.</summary>
    public System.Numerics.Vector3[] JointPositions;

    public int IterationsUsed;

    /// <summary>True only if the end joint landed within Tolerance of the target. False for the
    /// unreachable-target clamp, the target-at-root passthrough, and any case that used every
    /// available iteration without reaching tolerance - not an error signal, just "did not land
    /// exactly on target", matching the geometric reality of a rigid chain.</summary>
    public bool Converged;
}
