using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class PurityAndBindPoseTests
{
    [Fact]
    public void Solve_IsPure_SameInputSameOutput()
    {
        var input = TestHelpers.CanonicalInput();
        input.HasPole = true;
        input.PoleHint = new Vector3(0, 0, 1);
        input.TargetPosition = new Vector3(1.2f, 0.8f, 0.3f);

        var r1 = TwoBoneIkSolver.Solve(input);
        var r2 = TwoBoneIkSolver.Solve(input);

        Assert.Equal(r1.MidPosition, r2.MidPosition);
        Assert.Equal(r1.EndPosition, r2.EndPosition);
        Assert.Equal(r1.RootRotation, r2.RootRotation);
        Assert.Equal(r1.MidRotation, r2.MidRotation);
        Assert.Equal(r1.EndRotation, r2.EndRotation);
        Assert.Equal(r1.AppliedStretch, r2.AppliedStretch);
    }

    [Fact]
    public void Solve_IsScaleEquivariant()
    {
        var input = TestHelpers.CanonicalInput();
        input.HasPole = true;
        input.PoleHint = new Vector3(0, 0, 1);
        input.TargetPosition = new Vector3(1.2f, 0.8f, 0.3f);

        var scaled = input;
        const float scale = 100f;
        scaled.RootPosition *= scale;
        scaled.MidPosition *= scale;
        scaled.EndPosition *= scale;
        scaled.TargetPosition *= scale;
        scaled.PoleHint *= scale;

        var result = TwoBoneIkSolver.Solve(input);
        var resultScaled = TwoBoneIkSolver.Solve(scaled);

        Assert.True(((resultScaled.MidPosition - scaled.RootPosition) - scale * (result.MidPosition - input.RootPosition)).Length() < 1e-2f);
        Assert.True(((resultScaled.EndPosition - scaled.RootPosition) - scale * (result.EndPosition - input.RootPosition)).Length() < 1e-2f);
        Assert.True(TestHelpers.AngleBetweenDegrees(result.RootRotation, resultScaled.RootRotation) < 0.01f);
        Assert.True(TestHelpers.AngleBetweenDegrees(result.MidRotation, resultScaled.MidRotation) < 0.01f);
    }

    [Fact]
    public void AnalyzeBindPose_CanonicalChain_FindsElbowOffsetDirection()
    {
        var bind = TwoBoneIkSolver.AnalyzeBindPose(TestHelpers.CanonicalRoot, TestHelpers.CanonicalMid, TestHelpers.CanonicalEnd);

        Assert.True(bind.IsReliable);
        Assert.True(MathF.Abs(bind.RootBoneLength - (TestHelpers.CanonicalMid - TestHelpers.CanonicalRoot).Length()) < 1e-5f);
        Assert.True(MathF.Abs(bind.MidBoneLength - (TestHelpers.CanonicalEnd - TestHelpers.CanonicalMid).Length()) < 1e-5f);

        Vector3 chainDir = Vector3.Normalize(TestHelpers.CanonicalEnd - TestHelpers.CanonicalRoot);
        Vector3 elbowOffset = Vector3.Normalize(TestHelpers.ProjectPerpendicular(TestHelpers.CanonicalMid - TestHelpers.CanonicalRoot, chainDir));

        float dot = Math.Clamp(Vector3.Dot(elbowOffset, bind.DefaultPoleDirection), -1f, 1f);
        float angleErr = MathF.Acos(dot) * 180f / MathF.PI;
        Assert.True(angleErr < 0.5f, $"pole direction off by {angleErr} degrees");
        Assert.True(MathF.Abs(bind.DefaultPoleDirection.Length() - 1f) < 1e-5f);
    }

    [Fact]
    public void AnalyzeBindPose_StraightChain_IsUnreliableButDeterministic()
    {
        var root = new Vector3(0, 0, 0);
        var mid = new Vector3(1, 0, 0);
        var end = new Vector3(2, 0, 0);

        var bind1 = TwoBoneIkSolver.AnalyzeBindPose(root, mid, end);
        var bind2 = TwoBoneIkSolver.AnalyzeBindPose(root, mid, end);

        Assert.False(bind1.IsReliable);
        Assert.True(MathF.Abs(bind1.DefaultPoleDirection.Length() - 1f) < 1e-5f);
        Assert.True(MathF.Abs(Vector3.Dot(bind1.DefaultPoleDirection, Vector3.Normalize(end - root))) < 1e-4f, "pole direction not perpendicular to chain");
        Assert.Equal(bind1.DefaultPoleDirection, bind2.DefaultPoleDirection);
        Assert.Equal(bind1.BendNormal, bind2.BendNormal);
    }
}
