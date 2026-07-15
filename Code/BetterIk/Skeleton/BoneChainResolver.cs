#nullable enable

using System.Collections.Generic;

namespace BetterIk.Skeleton;

/// <summary>Resolves the root/mid/end bone chain for a two-bone IK from the end bone alone,
/// walking parent pointers, with optional manual overrides for root and mid.</summary>
public static class BoneChainResolver
{
    /// <summary>
    /// Auto-walk: mid = endBone.Parent, root = mid.Parent. rootOverride/midOverride, when
    /// non-null, are used verbatim instead of walking (the caller has already resolved
    /// override bone-name strings to nodes, or reported a not-found error itself - this
    /// function never does name lookup).
    /// </summary>
    public static BoneChainResult Resolve(IBoneNode? endBone, IBoneNode? rootOverride = null, IBoneNode? midOverride = null)
    {
        if (endBone is null)
            return BoneChainResult.Fail(BoneChainError.EndBoneMissing);

        IBoneNode? mid = midOverride ?? endBone.Parent;
        if (mid is null)
            return BoneChainResult.Fail(BoneChainError.InsufficientAncestors);

        IBoneNode? root = rootOverride ?? mid.Parent;
        if (root is null)
            return BoneChainResult.Fail(BoneChainError.InsufficientAncestors);

        if (root.Name == mid.Name || mid.Name == endBone.Name || root.Name == endBone.Name)
            return BoneChainResult.Fail(BoneChainError.DegenerateChain);

        return BoneChainResult.Ok(root, mid, endBone);
    }

    /// <summary>
    /// Walks up from endBone via .Parent, collecting every bone, until a bone named rootBoneName
    /// is found (name identity, matching this class's existing convention). Unlike Resolve, this
    /// does not assume a fixed chain depth - FABRIK chains are variable length, so the caller must
    /// name the root explicitly rather than relying on an auto-walk.
    /// </summary>
    public static FabrikChainResult ResolveNamedChain(IBoneNode? endBone, string rootBoneName)
    {
        if (endBone is null)
            return FabrikChainResult.Fail(FabrikChainError.EndBoneMissing);

        if (string.IsNullOrEmpty(rootBoneName))
            return FabrikChainResult.Fail(FabrikChainError.RootBoneNameMissing);

        var chain = new List<IBoneNode>();
        for (IBoneNode? node = endBone; node is not null; node = node.Parent)
        {
            chain.Add(node);
            if (node.Name == rootBoneName)
            {
                chain.Reverse();
                return chain.Count >= 2
                    ? FabrikChainResult.Ok(chain.ToArray())
                    : FabrikChainResult.Fail(FabrikChainError.ChainTooShort);
            }
        }

        return FabrikChainResult.Fail(FabrikChainError.RootNotFound);
    }

    /// <summary>Nearest common ancestor of a and b, where a node counts as its own ancestor.
    /// Null if the nodes belong to disjoint trees. Identity is by name, matching this class's
    /// existing name-based bone identity convention (a real adapter constructs a new wrapper
    /// instance per .Parent access, so reference equality would be wrong).</summary>
    public static IBoneNode? FindCommonAncestor(IBoneNode a, IBoneNode b)
    {
        var ancestorNames = new HashSet<string>();
        for (IBoneNode? node = a; node is not null; node = node.Parent)
            ancestorNames.Add(node.Name);

        for (IBoneNode? node = b; node is not null; node = node.Parent)
        {
            if (ancestorNames.Contains(node.Name))
                return node;
        }

        return null;
    }
}
