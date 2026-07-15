namespace BetterIk.Skeleton;

public enum FabrikChainError
{
    None,
    EndBoneMissing,
    RootBoneNameMissing,

    /// <summary>Walked all the way to the skeleton root without finding a bone named RootBone.</summary>
    RootNotFound,

    /// <summary>Root and end resolved to the same bone (by name) - a chain needs at least 2 bones.</summary>
    ChainTooShort,
}
