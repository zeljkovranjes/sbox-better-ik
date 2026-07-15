#nullable enable

namespace BetterIk.Skeleton;

/// <summary>Ordered root-first-to-end-last chain of bones for a variable-length FABRIK solve.</summary>
public readonly struct FabrikChainResult
{
    public readonly bool Success;
    public readonly FabrikChainError Error;

    /// <summary>Root-first, end-last. Null unless Success.</summary>
    public readonly IBoneNode[]? Chain;

    private FabrikChainResult(bool success, FabrikChainError error, IBoneNode[]? chain)
    {
        Success = success;
        Error = error;
        Chain = chain;
    }

    public static FabrikChainResult Ok(IBoneNode[] chain) => new(true, FabrikChainError.None, chain);
    public static FabrikChainResult Fail(FabrikChainError error) => new(false, error, null);
}
