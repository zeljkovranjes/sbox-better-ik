using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class ReachabilityTests
{
    [Fact]
    public void NearSideDeadZone_UnequalBoneLengths_ClampsToMinReach()
    {
        // l1 != l2, so there is a minimum reach |l1-l2| below which the target is unreachable
        // (the longer bone folding back over the shorter one is as close as the chain gets).
        var input = new TwoBoneIkInput
        {
            RootPosition = Vector3.Zero,
            MidPosition = new Vector3(0.3f, 0, 0),
            EndPosition = new Vector3(0.3f, 0.6f, 0),
            RootRotation = Quaternion.Identity,
            MidRotation = Quaternion.Identity,
            EndRotation = Quaternion.Identity,
            TargetRotation = Quaternion.Identity,
            HasPole = true,
            PoleHint = new Vector3(0, 0, 1),
            FallbackBendNormal = new Vector3(0, 0, 1),
            PositionWeight = 1f,
            RotationWeight = 1f,
            MasterWeight = 1f,
        };

        float l1 = 0.3f;
        float l2 = 0.6f;
        float minReach = MathF.Abs(l1 - l2);

        // Target well inside the dead zone (distance 0.05 from root, minReach is 0.3).
        input.TargetPosition = new Vector3(0.05f, 0, 0);
        var result = TwoBoneIkSolver.Solve(input);
        TestHelpers.AssertFinite(result);

        float endDist = (result.EndPosition - input.RootPosition).Length();
        Assert.True(MathF.Abs(endDist - minReach) < 1e-4f, $"expected clamp to minReach {minReach}, got {endDist}");
        Assert.True(MathF.Abs((result.MidPosition - input.RootPosition).Length() - l1) < 1e-4f);
        Assert.True(MathF.Abs((result.EndPosition - result.MidPosition).Length() - l2) < 1e-4f);

        // Target exactly at minReach: exact solve, no clamping needed.
        input.TargetPosition = new Vector3(minReach, 0, 0);
        var exactResult = TwoBoneIkSolver.Solve(input);
        TestHelpers.AssertFinite(exactResult);
        float exactErr = (exactResult.EndPosition - input.TargetPosition).Length();
        Assert.True(exactErr < 1e-4f, $"at exactly minReach, end error {exactErr}");
    }


    [Fact]
    public void BasicReachableSolve_PolePlaneInvariant()
    {
        var input = TestHelpers.CanonicalInput();
        input.HasPole = true;
        input.PoleHint = new Vector3(0, 0, 1);
        input.TargetPosition = new Vector3(1.2f, 0.8f, 0.3f);

        var result = TwoBoneIkSolver.Solve(input);
        TestHelpers.AssertFinite(result);

        float l1 = TestHelpers.L1(input);
        float l2 = TestHelpers.L2(input);

        Assert.True((result.EndPosition - input.TargetPosition).Length() < 1e-4f);
        Assert.True(MathF.Abs((result.MidPosition - input.RootPosition).Length() - l1) < 1e-4f);
        Assert.True(MathF.Abs((result.EndPosition - result.MidPosition).Length() - l2) < 1e-4f);

        Vector3 aHat = Vector3.Normalize(input.TargetPosition - input.RootPosition);
        Vector3 pPerp = TestHelpers.ProjectPerpendicular(input.PoleHint, aHat);
        Vector3 planeNormal = Vector3.Normalize(Vector3.Cross(aHat, pPerp));

        float outOfPlane = MathF.Abs(Vector3.Dot(result.MidPosition - input.RootPosition, planeNormal));
        Assert.True(outOfPlane < 1e-4f, $"elbow out of pole plane by {outOfPlane}");

        float sameSide = Vector3.Dot(result.MidPosition - input.RootPosition, Vector3.Normalize(pPerp));
        Assert.True(sameSide > 0f, "elbow not on pole side");

        // Root rotation is the delta (input rotation is identity), so it must reproduce MidPosition via FK.
        Vector3 reconMid = input.RootPosition + Vector3.Transform(input.MidPosition - input.RootPosition, result.RootRotation);
        Assert.True((reconMid - result.MidPosition).Length() < 1e-3f, $"rotation does not reproduce position, off by {(reconMid - result.MidPosition).Length()}");
    }

    [Fact]
    public void ReachabilityBoundarySweep()
    {
        var baseInput = TestHelpers.CanonicalInput();
        float lmax = TestHelpers.Lmax(baseInput);
        float l1 = TestHelpers.L1(baseInput);
        float l2 = TestHelpers.L2(baseInput);

        float[] ratios = { 0.5f, 0.9f, 0.99f, 1.0f, 1.01f, 1.5f, 3.0f };

        foreach (var ratio in ratios)
        {
            float d = ratio * lmax;
            var input = baseInput;
            input.HasPole = true;
            input.PoleHint = new Vector3(0, 0, 1);
            Vector3 dir = new Vector3(1, 0, 0);
            input.TargetPosition = input.RootPosition + dir * d;

            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            float midLen = (result.MidPosition - input.RootPosition).Length();
            float endLen = (result.EndPosition - result.MidPosition).Length();

            Assert.True(MathF.Abs(midLen - l1) < 1e-5f, $"ratio {ratio}: mid length {midLen} vs {l1}");
            Assert.True(MathF.Abs(endLen - l2) < 1e-5f, $"ratio {ratio}: end length {endLen} vs {l2}");

            if (d <= lmax)
            {
                float err = (result.EndPosition - input.TargetPosition).Length();
                Assert.True(err < 1e-4f * lmax, $"ratio {ratio}: end error {err}");
            }
            else
            {
                var expected = input.RootPosition + dir * lmax;
                float err = (result.EndPosition - expected).Length();
                Assert.True(err < 1e-4f, $"ratio {ratio}: overreach end error {err}");
            }
        }
    }
}
