namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;

/// <summary>Per-frame input to <see cref="FabrikSolver.Solve"/>. All positions are in one common
/// (typically world) space. Segment lengths are derived from JointPositions each solve - the
/// animated pose is the source of truth for lengths, same convention as the other solvers.</summary>
public struct FabrikInput
{
    /// <summary>N >= 2, world space, animated pose. Index 0 is the chain root.</summary>
    public System.Numerics.Vector3[] JointPositions;

    public Vector3 TargetPosition;

    /// <summary>0-1, clamped by the solver. 0 is an exact structural passthrough (no solve runs).</summary>
    public float Weight;

    public int MaxIterations;

    /// <summary>Absolute distance. The solver stops early once the end joint is within this of
    /// the target.</summary>
    public float Tolerance;
}
