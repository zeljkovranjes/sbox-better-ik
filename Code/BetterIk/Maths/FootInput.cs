namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>Per-foot input: animated pose plus the engine-supplied ground trace result. The
/// core never raycasts; the caller performs the scene trace and forwards the result here.</summary>
public struct FootInput
{
    public Vector3 FootPosition;
    public Quaternion FootRotation;

    public bool HasHit;

    /// <summary>World position. Only valid when <see cref="HasHit"/>.</summary>
    public Vector3 HitPoint;

    /// <summary>World normal, need not be normalized. Only valid when <see cref="HasHit"/>.</summary>
    public Vector3 HitNormal;
}
