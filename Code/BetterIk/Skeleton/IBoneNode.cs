#nullable enable

namespace BetterIk.Skeleton;

/// <summary>Minimal shape needed to walk a skeleton toward the root. Implemented by a real
/// BoneCollection.Bone adapter in the engine layer, and by a fake node in tests.</summary>
public interface IBoneNode
{
    string Name { get; }

    /// <summary>Null at the skeleton root.</summary>
    IBoneNode? Parent { get; }
}
