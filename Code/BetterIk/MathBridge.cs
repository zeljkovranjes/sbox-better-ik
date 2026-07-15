using Sandbox;

namespace BetterIk;

/// <summary>Direct field-for-field conversion between Sandbox's own math types and the
/// System.Numerics types used by the engine-agnostic solver core. Both represent the same
/// world space, so no basis-flip or handedness conversion is needed.</summary>
internal static class MathBridge
{
    public static System.Numerics.Vector3 ToNumerics(this Vector3 v) => new(v.x, v.y, v.z);
    public static Vector3 ToSandbox(this System.Numerics.Vector3 v) => new(v.X, v.Y, v.Z);

    public static System.Numerics.Quaternion ToNumerics(this Rotation r) => new(r.x, r.y, r.z, r.w);
    public static Rotation ToSandbox(this System.Numerics.Quaternion q) => new() { x = q.X, y = q.Y, z = q.Z, w = q.W };

    /// <summary>
    /// SetBoneTransform expects a model-local transform (relative to the renderer's own world
    /// transform), unlike TryGetBoneTransform/TryGetBoneTransformAnimation, which are documented
    /// as worldspace. Confirmed live: a character placed away from the world origin showed a
    /// bone landing at (world position + the character's own world offset) without this
    /// conversion - invisible in earlier testing only because every prior rig sat at world
    /// origin with identity rotation, where model-local and world space are numerically the same.
    /// </summary>
    public static global::Transform ToModelLocal(this SkinnedModelRenderer renderer, global::Transform worldTransform)
        => renderer.WorldTransform.ToLocal(worldTransform);
}
