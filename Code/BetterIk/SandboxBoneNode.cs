#nullable enable

using Sandbox;
using BetterIk.Skeleton;

namespace BetterIk;

/// <summary>Thin adapter over the real engine BoneCollection.Bone, exposing just enough to
/// let BoneChainResolver walk the skeleton. No logic beyond delegation - not unit tested.</summary>
internal sealed class SandboxBoneNode : IBoneNode
{
    public SandboxBoneNode(BoneCollection.Bone bone)
    {
        Bone = bone;
    }

    public BoneCollection.Bone Bone { get; }

    public string Name => Bone.Name;

    public IBoneNode? Parent => Bone.Parent is null ? null : new SandboxBoneNode(Bone.Parent);
}
