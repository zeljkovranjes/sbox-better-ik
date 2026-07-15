namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;
using Matrix4x4 = System.Numerics.Matrix4x4;

internal static class IkMath
{
    public static Vector3 SafeNormalize(Vector3 v, Vector3 fallback)
    {
        float lenSq = v.LengthSquared();
        if (lenSq < 1e-12f)
            return fallback;
        return v / MathF.Sqrt(lenSq);
    }

    public static Vector3 ProjectPerpendicular(Vector3 v, Vector3 unitAxis)
    {
        return v - Vector3.Dot(v, unitAxis) * unitAxis;
    }

    // Deterministic arbitrary perpendicular: cross with whichever world axis has the
    // smallest component along unitAxis, so the result never collapses to zero.
    public static Vector3 AnyPerpendicular(Vector3 unitAxis)
    {
        float ax = MathF.Abs(unitAxis.X);
        float ay = MathF.Abs(unitAxis.Y);
        float az = MathF.Abs(unitAxis.Z);

        Vector3 seed = (ax <= ay && ax <= az) ? Vector3.UnitX
            : (ay <= az) ? Vector3.UnitY
            : Vector3.UnitZ;

        Vector3 perp = Vector3.Cross(unitAxis, seed);
        if (perp.LengthSquared() < 1e-12f)
        {
            seed = seed == Vector3.UnitX ? Vector3.UnitY : Vector3.UnitX;
            perp = Vector3.Cross(unitAxis, seed);
        }

        return Vector3.Normalize(perp);
    }

    // U = dir, W = Gram-Schmidt of normalHint against U (fallback to AnyPerpendicular if degenerate), V = U x W.
    public static (Vector3 U, Vector3 W, Vector3 V) BuildOrthonormalBasis(Vector3 dir, Vector3 normalHint)
    {
        Vector3 u = SafeNormalize(dir, Vector3.UnitX);
        Vector3 wRaw = ProjectPerpendicular(normalHint, u);
        Vector3 w = wRaw.LengthSquared() < 1e-10f ? AnyPerpendicular(u) : Vector3.Normalize(wRaw);
        Vector3 v = Vector3.Cross(u, w);
        return (u, w, v);
    }

    // Rotation mapping the old (dir, normalHint) frame exactly onto the new (dir, normalHint) frame:
    // delta * oldU = newU, delta * oldW = newW, delta * oldV = newV. Built via change-of-basis matrices
    // rather than shortest-arc, so twist/roll is pinned and there is no 180-degree flip ambiguity.
    public static Quaternion DeltaRotation(Vector3 oldDir, Vector3 oldNormalHint, Vector3 newDir, Vector3 newNormalHint)
    {
        var (uOld, wOld, vOld) = BuildOrthonormalBasis(oldDir, oldNormalHint);
        var (uNew, wNew, vNew) = BuildOrthonormalBasis(newDir, newNormalHint);

        // Row-vector convention (matches System.Numerics Vector3.Transform(v, Matrix4x4)):
        // row i of the matrix is where local axis i maps to.
        var mOld = new Matrix4x4(
            uOld.X, uOld.Y, uOld.Z, 0f,
            wOld.X, wOld.Y, wOld.Z, 0f,
            vOld.X, vOld.Y, vOld.Z, 0f,
            0f, 0f, 0f, 1f);

        var mNew = new Matrix4x4(
            uNew.X, uNew.Y, uNew.Z, 0f,
            wNew.X, wNew.Y, wNew.Z, 0f,
            vNew.X, vNew.Y, vNew.Z, 0f,
            0f, 0f, 0f, 1f);

        // mOld is orthonormal, so its inverse is its transpose: delta = mOld^-1 * mNew.
        var delta = Matrix4x4.Transpose(mOld) * mNew;
        return Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(delta));
    }
}
