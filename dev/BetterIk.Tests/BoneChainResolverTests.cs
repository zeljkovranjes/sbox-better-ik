using BetterIk.Skeleton;
using Xunit;

namespace BetterIk.Tests;

internal sealed class FakeBoneNode : IBoneNode
{
    public string Name { get; }
    public IBoneNode? Parent { get; }

    public FakeBoneNode(string name, IBoneNode? parent = null)
    {
        Name = name;
        Parent = parent;
    }
}

public class BoneChainResolverTests
{
    [Fact]
    public void HappyPath_ThreeOrMoreAncestors_Succeeds()
    {
        var greatGrandparent = new FakeBoneNode("pelvis");
        var root = new FakeBoneNode("upper_arm", greatGrandparent);
        var mid = new FakeBoneNode("forearm", root);
        var end = new FakeBoneNode("hand", mid);

        var result = BoneChainResolver.Resolve(end);

        Assert.True(result.Success);
        Assert.Equal(BoneChainError.None, result.Error);
        Assert.Same(root, result.Root);
        Assert.Same(mid, result.Mid);
        Assert.Same(end, result.End);
    }

    [Fact]
    public void ExactlyTwoAncestors_RootHasNullParent_Succeeds()
    {
        var root = new FakeBoneNode("upper_arm", null);
        var mid = new FakeBoneNode("forearm", root);
        var end = new FakeBoneNode("hand", mid);

        var result = BoneChainResolver.Resolve(end);

        Assert.True(result.Success);
        Assert.Same(root, result.Root);
        Assert.Same(mid, result.Mid);
        Assert.Same(end, result.End);
    }

    [Fact]
    public void OneAncestor_FailsWithInsufficientAncestors()
    {
        var mid = new FakeBoneNode("forearm", null);
        var end = new FakeBoneNode("hand", mid);

        var result = BoneChainResolver.Resolve(end);

        Assert.False(result.Success);
        Assert.Equal(BoneChainError.InsufficientAncestors, result.Error);
    }

    [Fact]
    public void ZeroAncestors_FailsWithInsufficientAncestors()
    {
        var end = new FakeBoneNode("hand", null);

        var result = BoneChainResolver.Resolve(end);

        Assert.False(result.Success);
        Assert.Equal(BoneChainError.InsufficientAncestors, result.Error);
    }

    [Fact]
    public void EndBoneNull_FailsWithEndBoneMissing()
    {
        var result = BoneChainResolver.Resolve(null);

        Assert.False(result.Success);
        Assert.Equal(BoneChainError.EndBoneMissing, result.Error);
    }

    [Fact]
    public void RootOverride_AutoMid_Succeeds()
    {
        var mid = new FakeBoneNode("forearm", null);
        var end = new FakeBoneNode("hand", mid);
        var rootOverride = new FakeBoneNode("clavicle");

        var result = BoneChainResolver.Resolve(end, rootOverride: rootOverride);

        Assert.True(result.Success);
        Assert.Same(rootOverride, result.Root);
        Assert.Same(mid, result.Mid);
        Assert.Same(end, result.End);
    }

    [Fact]
    public void BothOverrides_NeverTouchesEndBoneParent_Succeeds()
    {
        var end = new FakeBoneNode("hand", null);
        var rootOverride = new FakeBoneNode("clavicle");
        var midOverride = new FakeBoneNode("forearm");

        var result = BoneChainResolver.Resolve(end, rootOverride: rootOverride, midOverride: midOverride);

        Assert.True(result.Success);
        Assert.Same(rootOverride, result.Root);
        Assert.Same(midOverride, result.Mid);
        Assert.Same(end, result.End);
    }

    [Fact]
    public void MidOverrideSameNameAsEndBone_FailsWithDegenerateChain()
    {
        var end = new FakeBoneNode("hand", null);
        var rootOverride = new FakeBoneNode("clavicle");
        var midOverride = new FakeBoneNode("hand"); // same name as end bone

        var result = BoneChainResolver.Resolve(end, rootOverride: rootOverride, midOverride: midOverride);

        Assert.False(result.Success);
        Assert.Equal(BoneChainError.DegenerateChain, result.Error);
    }

    [Fact]
    public void RootOverrideSameNameAsMid_FailsWithDegenerateChain()
    {
        var mid = new FakeBoneNode("forearm", null);
        var end = new FakeBoneNode("hand", mid);
        var rootOverride = new FakeBoneNode("forearm"); // same name as auto-resolved mid

        var result = BoneChainResolver.Resolve(end, rootOverride: rootOverride);

        Assert.False(result.Success);
        Assert.Equal(BoneChainError.DegenerateChain, result.Error);
    }

    [Fact]
    public void NameEquality_NotReferenceEquality_DeterminesIdentity()
    {
        // Two distinct instances with the same name are treated as the same bone, matching
        // BoneCollection.HasBone/GetBone's name-based identity contract - a real adapter may
        // construct a new wrapper instance per .Parent access.
        var end = new FakeBoneNode("hand", null);
        var rootOverride = new FakeBoneNode("clavicle");
        var midOverride = new FakeBoneNode("hand"); // distinct instance, same name as end

        var result = BoneChainResolver.Resolve(end, rootOverride: rootOverride, midOverride: midOverride);

        Assert.False(result.Success);
        Assert.Equal(BoneChainError.DegenerateChain, result.Error);
    }
}
