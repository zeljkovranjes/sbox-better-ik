namespace BetterIk.Maths;

using System.Collections.Generic;
using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>
/// Engine-agnostic orientation-warp math: the signed horizontal turn angle between a movement
/// direction and the character's forward, clamped to a maximum, the per-bone counter-rotation
/// factor table used to apply it (full below the spine chain, tapering to zero at its top), and
/// the rigid per-bone rotation about a pivot. Pure and stateless: identical input always produces
/// identical output, safe to call every frame.
/// </summary>
public static class OrientationWarpSolver
{
    private const float TinyLenSq = 1e-12f;

    /// <summary>Signed angle (radians) from `forward` to `movementDirection`, measured in the plane
    /// perpendicular to `up`, clamped to +/-maxWarpRadians. 0 when `up` is degenerate or either
    /// direction has no component perpendicular to it.</summary>
    public static float ComputeClampedAngle(Vector3 forward, Vector3 movementDirection, Vector3 up, float maxWarpRadians)
    {
        if (up.LengthSquared() < TinyLenSq)
            return 0f;
        Vector3 unitUp = Vector3.Normalize(up);

        Vector3 f = IkMath.ProjectPerpendicular(forward, unitUp);
        Vector3 m = IkMath.ProjectPerpendicular(movementDirection, unitUp);
        if (f.LengthSquared() < TinyLenSq || m.LengthSquared() < TinyLenSq)
            return 0f;

        f = Vector3.Normalize(f);
        m = Vector3.Normalize(m);

        float cosAngle = Math.Clamp(Vector3.Dot(f, m), -1f, 1f);
        float angle = MathF.Acos(cosAngle);
        float sign = Vector3.Dot(Vector3.Cross(f, m), unitUp) < 0f ? -1f : 1f;

        float maxAngle = MathF.Max(maxWarpRadians, 0f);
        return Math.Clamp(angle * sign, -maxAngle, maxAngle);
    }

    /// <summary>Rotates a bone's position/rotation by `angle` radians around `up`, pivoting at
    /// `pivot`. Identity passthrough at zero angle or a degenerate up axis.</summary>
    public static (Vector3 Position, Quaternion Rotation) RotateAroundPivot(Vector3 position, Quaternion rotation, Vector3 pivot, Vector3 up, float angle)
    {
        if (MathF.Abs(angle) < 1e-9f || up.LengthSquared() < TinyLenSq)
            return (position, rotation);

        Quaternion delta = Quaternion.CreateFromAxisAngle(Vector3.Normalize(up), angle);
        Vector3 newPosition = pivot + Vector3.Transform(position - pivot, delta);
        Quaternion newRotation = Quaternion.Normalize(delta * rotation);
        return (newPosition, newRotation);
    }

    /// <summary>
    /// Per-bone counter-rotation factor (0..1), keyed by bone index. `spineBoneNames` is an ordered
    /// chain from nearest-pelvis to nearest-head; each named entry gets a linearly decreasing factor
    /// (1.0 at the first entry down to 0.0 at the last), and every other bone inherits its nearest
    /// named-spine ancestor's factor by walking `parentIndices` toward the root. A bone with no
    /// named-spine ancestor (legs, pelvis, or anything outside the chain) gets 1.0 (full
    /// counter-rotation); a bone descended from the last named entry (e.g. head, arms) inherits 0.0.
    /// </summary>
    public static Dictionary<int, float> ComputeBoneFactors(IReadOnlyList<string> boneNames, IReadOnlyList<int> parentIndices, IReadOnlyList<string> spineBoneNames)
    {
        var spineFactorByName = new Dictionary<string, float>(spineBoneNames.Count);
        int spineCount = spineBoneNames.Count;
        for (int i = 0; i < spineCount; i++)
        {
            float factor = spineCount <= 1 ? 0f : 1f - (float)i / (spineCount - 1);
            spineFactorByName[spineBoneNames[i]] = factor;
        }

        var result = new Dictionary<int, float>(boneNames.Count);
        for (int i = 0; i < boneNames.Count; i++)
            result[i] = ResolveFactor(i, boneNames, parentIndices, spineFactorByName);

        return result;
    }

    private static float ResolveFactor(int boneIndex, IReadOnlyList<string> boneNames, IReadOnlyList<int> parentIndices, Dictionary<string, float> spineFactorByName)
    {
        int current = boneIndex;
        while (current >= 0)
        {
            if (spineFactorByName.TryGetValue(boneNames[current], out float factor))
                return factor;
            current = parentIndices[current];
        }
        return 1f;
    }
}
