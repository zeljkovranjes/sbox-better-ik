namespace BetterIk.Skeleton;

public enum BoneChainError
{
    None,
    EndBoneMissing,

    /// <summary>The end bone has fewer than 2 ancestors via auto-walk.</summary>
    InsufficientAncestors,

    /// <summary>Root, mid, and end resolved to fewer than 3 distinct bones (by name).</summary>
    DegenerateChain,
}
