namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>Output of <see cref="TwoBoneIkSolver.Solve"/>, already blended by the input weights.</summary>
public struct TwoBoneIkResult
{
    public Quaternion RootRotation;
    public Quaternion MidRotation;
    public Quaternion EndRotation;
    public Vector3 MidPosition;
    public Vector3 EndPosition;

    /// <summary>Blended uniform bone-length scale actually applied. 1 when no stretch occurred.</summary>
    public float AppliedStretch;

    /// <summary>False when the chain was degenerate (near-zero bone length) and the pose was passed through unmodified.</summary>
    public bool Solved;
}
