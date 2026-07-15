using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class DegenerateTests
{
    [Fact]
    public void ZeroLengthRootBone_PassesThroughButStillBlendsRotation()
    {
        var input = TestHelpers.CanonicalInput();
        input.MidPosition = input.RootPosition; // L1 = 0
        input.TargetPosition = new Vector3(5, 1, 1);
        input.TargetRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 1f);
        input.RotationWeight = 0.7f;

        var result = TwoBoneIkSolver.Solve(input);
        TestHelpers.AssertFinite(result);

        Assert.False(result.Solved);
        Assert.True(TestHelpers.AngleBetweenDegrees(result.RootRotation, input.RootRotation) < 0.001f);
        Assert.True(TestHelpers.AngleBetweenDegrees(result.MidRotation, input.MidRotation) < 0.001f);

        var expectedEndRotation = Quaternion.Slerp(input.EndRotation, input.TargetRotation, 0.7f);
        Assert.True(TestHelpers.AngleBetweenDegrees(result.EndRotation, expectedEndRotation) < 0.01f);
    }

    [Fact]
    public void TargetAtRoot_EqualLengthChain_FoldsWithoutNaN()
    {
        var input = TestHelpers.CanonicalInput(); // L1 == L2 (equal-length canonical chain)
        input.TargetPosition = input.RootPosition;
        input.HasPole = true;
        input.PoleHint = new Vector3(0, 0, 1);

        var result = TwoBoneIkSolver.Solve(input);
        TestHelpers.AssertFinite(result);

        float err = (result.EndPosition - input.TargetPosition).Length();
        Assert.True(err < 1e-3f, $"end error {err}");
    }
}
