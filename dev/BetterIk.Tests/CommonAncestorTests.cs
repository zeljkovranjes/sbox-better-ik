using BetterIk.Skeleton;
using Xunit;

namespace BetterIk.Tests;

public class CommonAncestorTests
{
    [Fact]
    public void TwoLegRoots_ShareParent_ReturnsPelvis()
    {
        var pelvis = new FakeBoneNode("pelvis");
        var legRootL = new FakeBoneNode("leg_upper_L", pelvis);
        var legRootR = new FakeBoneNode("leg_upper_R", pelvis);

        var ancestor = BoneChainResolver.FindCommonAncestor(legRootL, legRootR);

        Assert.NotNull(ancestor);
        Assert.Equal("pelvis", ancestor!.Name);
    }

    [Fact]
    public void NodeIsItsOwnAncestor()
    {
        var root = new FakeBoneNode("pelvis");
        var descendant = new FakeBoneNode("leg_upper_L", root);

        var ancestor = BoneChainResolver.FindCommonAncestor(root, descendant);

        Assert.NotNull(ancestor);
        Assert.Equal(root.Name, ancestor!.Name);
    }

    [Fact]
    public void DisjointTrees_ReturnsNull()
    {
        var treeA = new FakeBoneNode("a_root");
        var aChild = new FakeBoneNode("a_child", treeA);

        var treeB = new FakeBoneNode("b_root");
        var bChild = new FakeBoneNode("b_child", treeB);

        var ancestor = BoneChainResolver.FindCommonAncestor(aChild, bChild);

        Assert.Null(ancestor);
    }

    [Fact]
    public void SameNode_ReturnsItself()
    {
        var node = new FakeBoneNode("hand");

        var ancestor = BoneChainResolver.FindCommonAncestor(node, node);

        Assert.NotNull(ancestor);
        Assert.Equal(node.Name, ancestor!.Name);
    }
}
