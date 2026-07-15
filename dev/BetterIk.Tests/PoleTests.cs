using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class PoleTests
{
    [Fact]
    public void DegeneratePole_CollinearWithAxis_FallsBackToPoseDirection()
    {
        var input = TestHelpers.CanonicalInput();
        input.TargetPosition = new Vector3(1.6f, 0f, 0.2f);
        Vector3 aHat = Vector3.Normalize(input.TargetPosition - input.RootPosition);

        input.HasPole = true;
        input.PoleHint = aHat * 2f;

        var result = TwoBoneIkSolver.Solve(input);
        TestHelpers.AssertFinite(result);
        Assert.True((result.EndPosition - input.TargetPosition).Length() < 1e-4f);

        Vector3 poseDir = Vector3.Normalize(TestHelpers.ProjectPerpendicular(input.MidPosition - input.RootPosition, aHat));
        Vector3 actualDir = Vector3.Normalize(TestHelpers.ProjectPerpendicular(result.MidPosition - input.RootPosition, aHat));
        float dot = Math.Clamp(Vector3.Dot(poseDir, actualDir), -1f, 1f);
        float angleErr = MathF.Acos(dot) * 180f / MathF.PI;
        Assert.True(angleErr < 1f, $"degenerate pole: elbow off pose direction by {angleErr} degrees");

        // Near-degenerate (0.005 degrees off axis): must still be finite with lengths preserved.
        var input2 = input;
        var slightlyOff = Vector3.Transform(aHat, Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.005f * MathF.PI / 180f));
        input2.PoleHint = slightlyOff * 2f;
        var result2 = TwoBoneIkSolver.Solve(input2);
        TestHelpers.AssertFinite(result2);
        float l1 = TestHelpers.L1(input2);
        float l2 = TestHelpers.L2(input2);
        Assert.True(MathF.Abs((result2.MidPosition - input2.RootPosition).Length() - l1) < 1e-4f);
        Assert.True(MathF.Abs((result2.EndPosition - result2.MidPosition).Length() - l2) < 1e-4f);
    }

    [Fact]
    public void CollinearAnimatedPose_WithPole_BendsTowardPole()
    {
        var input = TestHelpers.CanonicalInput();
        input.RootPosition = new Vector3(0, 0, 0);
        input.MidPosition = new Vector3(1, 0, 0);
        input.EndPosition = new Vector3(2, 0, 0);
        input.FallbackBendNormal = new Vector3(0, 0, 1);
        input.TargetPosition = new Vector3(1, 1, 0);
        input.HasPole = true;
        input.PoleHint = new Vector3(0, 0, 1);

        var result = TwoBoneIkSolver.Solve(input);
        TestHelpers.AssertFinite(result);
        Assert.True((result.EndPosition - input.TargetPosition).Length() < 1e-4f);

        Vector3 aHat = Vector3.Normalize(input.TargetPosition - input.RootPosition);
        Vector3 elbowDir = Vector3.Normalize(TestHelpers.ProjectPerpendicular(result.MidPosition - input.RootPosition, aHat));
        Assert.True(elbowDir.Z > 0.9f, $"elbow did not bend toward +Z pole, dir={elbowDir}");
    }

    [Fact]
    public void CollinearAnimatedPose_NoPole_UsesFallbackNormalLadder()
    {
        var input = TestHelpers.CanonicalInput();
        input.RootPosition = new Vector3(0, 0, 0);
        input.MidPosition = new Vector3(1, 0, 0);
        input.EndPosition = new Vector3(2, 0, 0);
        input.FallbackBendNormal = new Vector3(0, 0, 1);
        input.TargetPosition = new Vector3(1, 1, 0);
        input.HasPole = false;

        var result = TwoBoneIkSolver.Solve(input);
        TestHelpers.AssertFinite(result);
        Assert.True((result.EndPosition - input.TargetPosition).Length() < 1e-4f);
    }
}
