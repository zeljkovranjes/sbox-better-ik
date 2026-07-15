using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

internal static class TestHelpers
{
    public static readonly Vector3 CanonicalRoot = new(0, 0, 0);
    public static readonly Vector3 CanonicalMid = new(1, 0, 0.1f);
    public static readonly Vector3 CanonicalEnd = new(2, 0, 0);

    public static TwoBoneIkInput CanonicalInput()
    {
        var bind = TwoBoneIkSolver.AnalyzeBindPose(CanonicalRoot, CanonicalMid, CanonicalEnd);
        return new TwoBoneIkInput
        {
            RootPosition = CanonicalRoot,
            MidPosition = CanonicalMid,
            EndPosition = CanonicalEnd,
            RootRotation = Quaternion.Identity,
            MidRotation = Quaternion.Identity,
            EndRotation = Quaternion.Identity,
            TargetPosition = CanonicalEnd,
            TargetRotation = Quaternion.Identity,
            HasPole = false,
            PoleHint = Vector3.Zero,
            PoleAngleOffsetRadians = 0f,
            FallbackBendNormal = bind.BendNormal,
            PositionWeight = 1f,
            RotationWeight = 1f,
            MasterWeight = 1f,
            SoftFraction = 0f,
            MaxStretch = 0f,
        };
    }

    public static float Lmax(TwoBoneIkInput input)
        => (input.MidPosition - input.RootPosition).Length() + (input.EndPosition - input.MidPosition).Length();

    public static float L1(TwoBoneIkInput input) => (input.MidPosition - input.RootPosition).Length();
    public static float L2(TwoBoneIkInput input) => (input.EndPosition - input.MidPosition).Length();

    public static float AngleBetweenDegrees(Quaternion a, Quaternion b)
    {
        a = Quaternion.Normalize(a);
        b = Quaternion.Normalize(b);
        float dot = Math.Clamp(MathF.Abs(Quaternion.Dot(a, b)), -1f, 1f);
        return 2f * MathF.Acos(dot) * (180f / MathF.PI);
    }

    public static Vector3 ProjectPerpendicular(Vector3 v, Vector3 axis)
        => v - Vector3.Dot(v, axis) * axis;

    public static void AssertFinite(TwoBoneIkResult r)
    {
        AssertFiniteVector(r.MidPosition, nameof(r.MidPosition));
        AssertFiniteVector(r.EndPosition, nameof(r.EndPosition));
        AssertFiniteQuaternion(r.RootRotation, nameof(r.RootRotation));
        AssertFiniteQuaternion(r.MidRotation, nameof(r.MidRotation));
        AssertFiniteQuaternion(r.EndRotation, nameof(r.EndRotation));
        Assert.False(float.IsNaN(r.AppliedStretch) || float.IsInfinity(r.AppliedStretch), "AppliedStretch not finite");
    }

    private static void AssertFiniteVector(Vector3 v, string name)
    {
        Assert.False(float.IsNaN(v.X) || float.IsNaN(v.Y) || float.IsNaN(v.Z)
            || float.IsInfinity(v.X) || float.IsInfinity(v.Y) || float.IsInfinity(v.Z), $"{name} not finite: {v}");
    }

    private static void AssertFiniteQuaternion(Quaternion q, string name)
    {
        Assert.False(float.IsNaN(q.X) || float.IsNaN(q.Y) || float.IsNaN(q.Z) || float.IsNaN(q.W)
            || float.IsInfinity(q.X) || float.IsInfinity(q.Y) || float.IsInfinity(q.Z) || float.IsInfinity(q.W), $"{name} not finite: {q}");
    }
}
