using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class WeightBlendingTests
{
    [Fact]
    public void MasterWeightSweep_ContinuityAndNoSquash()
    {
        var baseInput = TestHelpers.CanonicalInput();
        baseInput.HasPole = true;
        baseInput.PoleHint = new Vector3(0, 0, 1);
        baseInput.TargetPosition = new Vector3(1.2f, 0.8f, 0.3f);
        baseInput.TargetRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI / 2f);

        float l1 = TestHelpers.L1(baseInput);
        float l2 = TestHelpers.L2(baseInput);
        float lmax = TestHelpers.Lmax(baseInput);

        TwoBoneIkResult? prev = null;

        for (int i = 0; i <= 100; i++)
        {
            float w = i / 100f;
            var input = baseInput;
            input.MasterWeight = w;
            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            float midLen = (result.MidPosition - input.RootPosition).Length();
            float endLen = (result.EndPosition - result.MidPosition).Length();
            Assert.True(MathF.Abs(midLen - l1) < 1e-4f, $"w={w}: mid length {midLen} vs {l1}");
            Assert.True(MathF.Abs(endLen - l2) < 1e-4f, $"w={w}: end length {endLen} vs {l2}");

            if (i == 0)
            {
                Assert.True((result.MidPosition - input.MidPosition).Length() < 1e-5f, "w=0 mid position not passthrough");
                Assert.True((result.EndPosition - input.EndPosition).Length() < 1e-5f, "w=0 end position not passthrough");
                Assert.True(TestHelpers.AngleBetweenDegrees(result.RootRotation, input.RootRotation) < 0.01f, "w=0 root rotation not passthrough");
                Assert.True(TestHelpers.AngleBetweenDegrees(result.EndRotation, input.EndRotation) < 0.01f, "w=0 end rotation not passthrough");
            }
            if (i == 100)
            {
                Assert.True((result.EndPosition - input.TargetPosition).Length() < 1e-4f, "w=1 end not on target");
            }

            if (prev.HasValue)
            {
                float posMove = (result.EndPosition - prev.Value.EndPosition).Length();
                Assert.True(posMove < 0.02f * lmax, $"w={w}: end position jump {posMove}");
                float rotMove = TestHelpers.AngleBetweenDegrees(result.RootRotation, prev.Value.RootRotation);
                Assert.True(rotMove < 2f, $"w={w}: root rotation jump {rotMove}");
            }

            prev = result;
        }
    }

    [Fact]
    public void MasterTimesComponentWeight_EqualsProduct()
    {
        var baseInput = TestHelpers.CanonicalInput();
        baseInput.HasPole = true;
        baseInput.PoleHint = new Vector3(0, 0, 1);
        baseInput.TargetPosition = new Vector3(1.2f, 0.8f, 0.3f);
        baseInput.TargetRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI / 2f);

        var inputA = baseInput;
        inputA.MasterWeight = 0.5f;
        inputA.PositionWeight = 1f;
        inputA.RotationWeight = 1f;

        var inputB = baseInput;
        inputB.MasterWeight = 1f;
        inputB.PositionWeight = 0.5f;
        inputB.RotationWeight = 0.5f;

        var resultA = TwoBoneIkSolver.Solve(inputA);
        var resultB = TwoBoneIkSolver.Solve(inputB);

        Assert.True((resultA.MidPosition - resultB.MidPosition).Length() < 1e-5f);
        Assert.True((resultA.EndPosition - resultB.EndPosition).Length() < 1e-5f);
        Assert.True(TestHelpers.AngleBetweenDegrees(resultA.RootRotation, resultB.RootRotation) < 0.001f);
        Assert.True(TestHelpers.AngleBetweenDegrees(resultA.MidRotation, resultB.MidRotation) < 0.001f);
        Assert.True(TestHelpers.AngleBetweenDegrees(resultA.EndRotation, resultB.EndRotation) < 0.001f);
    }

    [Fact]
    public void RotationWeight_IsIndependentOfPositionSolve()
    {
        var baseInput = TestHelpers.CanonicalInput();
        baseInput.HasPole = true;
        baseInput.PoleHint = new Vector3(0, 0, 1);
        baseInput.TargetPosition = new Vector3(1.2f, 0.8f, 0.3f);
        baseInput.TargetRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI / 2f);
        baseInput.PositionWeight = 1f;

        TwoBoneIkResult? reference = null;

        foreach (var wRot in new[] { 0f, 0.25f, 0.5f, 1f })
        {
            var input = baseInput;
            input.RotationWeight = wRot;
            var result = TwoBoneIkSolver.Solve(input);

            var expectedEndRotation = wRot <= 0f ? input.EndRotation
                : wRot >= 1f ? input.TargetRotation
                : Quaternion.Slerp(input.EndRotation, input.TargetRotation, wRot);
            Assert.True(TestHelpers.AngleBetweenDegrees(result.EndRotation, expectedEndRotation) < 0.01f, $"wRot={wRot}: end rotation mismatch");

            if (reference.HasValue)
            {
                Assert.True((result.MidPosition - reference.Value.MidPosition).Length() < 1e-6f, $"wRot={wRot}: mid position changed");
                Assert.True((result.EndPosition - reference.Value.EndPosition).Length() < 1e-6f, $"wRot={wRot}: end position changed");
                Assert.True(TestHelpers.AngleBetweenDegrees(result.RootRotation, reference.Value.RootRotation) < 0.001f, $"wRot={wRot}: root rotation changed");
                Assert.True(TestHelpers.AngleBetweenDegrees(result.MidRotation, reference.Value.MidRotation) < 0.001f, $"wRot={wRot}: mid rotation changed");
            }
            reference = result;
        }
    }
}
