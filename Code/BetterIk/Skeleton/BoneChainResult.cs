#nullable enable

namespace BetterIk.Skeleton;

public readonly struct BoneChainResult
{
    public readonly bool Success;
    public readonly BoneChainError Error;
    public readonly IBoneNode? Root;
    public readonly IBoneNode? Mid;
    public readonly IBoneNode? End;

    private BoneChainResult(bool success, BoneChainError error, IBoneNode? root, IBoneNode? mid, IBoneNode? end)
    {
        Success = success;
        Error = error;
        Root = root;
        Mid = mid;
        End = end;
    }

    public static BoneChainResult Ok(IBoneNode root, IBoneNode mid, IBoneNode end)
        => new(true, BoneChainError.None, root, mid, end);

    public static BoneChainResult Fail(BoneChainError error)
        => new(false, error, null, null, null);
}
