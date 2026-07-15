using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class FootPlacementTests
{
    private static readonly Vector3 Up = Vector3.UnitZ;

    private static FootInput Grounded(Vector3 footPosition, Quaternion footRotation, Vector3 hitPoint, Vector3 hitNormal)
        => new()
        {
            FootPosition = footPosition,
            FootRotation = footRotation,
            HasHit = true,
            HitPoint = hitPoint,
            HitNormal = hitNormal,
        };

    private static FootInput Ungrounded(Vector3 footPosition, Quaternion footRotation)
        => new()
        {
            FootPosition = footPosition,
            FootRotation = footRotation,
            HasHit = false,
        };

    private static FootPlacementInput BaseInput(Vector3 up)
        => new()
        {
            OriginPosition = Vector3.Zero,
            UpAxis = up,
            FootHeightOffset = 0f,
            MaxStepUp = 18f,
            MaxStepDown = 18f,
            MaxPelvisDrop = 16f,
            MaxPelvisRaise = 0f,
            MaxFootRotationRadians = 30f * (MathF.PI / 180f),
            MaxGroundSlopeRadians = 60f * (MathF.PI / 180f),
            Weight = 1f,
        };

    [Fact]
    public void FlatGroundNoOp()
    {
        var input = BaseInput(Up);
        input.LeftFoot = Grounded(new Vector3(-5, 0, 0), Quaternion.Identity, new Vector3(-5, 0, 0), Up);
        input.RightFoot = Grounded(new Vector3(5, 0, 0), Quaternion.Identity, new Vector3(5, 0, 0), Up);

        var result = FootPlacementSolver.Solve(input);

        Assert.True(result.Solved);
        Assert.True(result.LeftFoot.Grounded);
        Assert.True(result.RightFoot.Grounded);
        Assert.Equal(0f, result.LeftFoot.VerticalDelta, 5);
        Assert.Equal(0f, result.RightFoot.VerticalDelta, 5);
        Assert.Equal(0f, result.PelvisOffset, 5);
        Assert.True(TestHelpers.AngleBetweenDegrees(result.LeftFoot.TargetRotation, Quaternion.Identity) < 0.01f);
        Assert.True(TestHelpers.AngleBetweenDegrees(result.RightFoot.TargetRotation, Quaternion.Identity) < 0.01f);
    }

    [Fact]
    public void StepUpWithinAndBeyondClamp_NonUnitZUp()
    {
        Vector3 up = Vector3.UnitX;
        var input = BaseInput(up);
        input.MaxStepUp = 18f;

        input.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, up * 10f, up);
        input.RightFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);
        var withinResult = FootPlacementSolver.Solve(input);
        Assert.True(withinResult.LeftFoot.Grounded);
        Assert.Equal(10f, withinResult.LeftFoot.VerticalDelta, 4);

        input.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, up * 25f, up);
        var beyondResult = FootPlacementSolver.Solve(input);
        Assert.True(beyondResult.LeftFoot.Grounded);
        Assert.Equal(18f, beyondResult.LeftFoot.VerticalDelta, 4);
    }

    [Fact]
    public void StepDownPelvisDrop()
    {
        var input = BaseInput(Up);
        input.Weight = 1f;
        input.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, Vector3.Zero, Up);
        input.RightFoot = Grounded(Vector3.Zero, Quaternion.Identity, Up * -10f, Up);

        var result = FootPlacementSolver.Solve(input);

        Assert.Equal(0f, result.LeftFoot.VerticalDelta, 4);
        Assert.Equal(-10f, result.RightFoot.VerticalDelta, 4);
        Assert.Equal(-10f, result.PelvisOffset, 4);
    }

    [Fact]
    public void PelvisDropClamped()
    {
        var input = BaseInput(Up);
        input.MaxStepDown = 18f;
        input.MaxPelvisDrop = 16f;
        input.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, Up * -50f, Up);
        input.RightFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);

        var result = FootPlacementSolver.Solve(input);

        Assert.Equal(-18f, result.LeftFoot.VerticalDelta, 4);
        Assert.Equal(-16f, result.PelvisOffset, 4);
    }

    [Fact]
    public void UngroundedFootExcluded()
    {
        var input = BaseInput(Up);
        input.LeftFoot = Ungrounded(new Vector3(1, 2, 3), new Quaternion(0.1f, 0.2f, 0.3f, 0.9274f));
        input.RightFoot = Grounded(Vector3.Zero, Quaternion.Identity, Up * -8f, Up);

        var result = FootPlacementSolver.Solve(input);

        Assert.False(result.LeftFoot.Grounded);
        Assert.Equal(0f, result.LeftFoot.VerticalDelta);
        Assert.Equal(input.LeftFoot.FootRotation, result.LeftFoot.TargetRotation);
        Assert.Equal(-8f, result.PelvisOffset, 4);
    }

    [Fact]
    public void BothUngrounded_PelvisZero()
    {
        var input = BaseInput(Up);
        input.LeftFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);
        input.RightFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);

        var result = FootPlacementSolver.Solve(input);

        Assert.True(result.Solved);
        Assert.False(result.LeftFoot.Grounded);
        Assert.False(result.RightFoot.Grounded);
        Assert.Equal(0f, result.PelvisOffset);
    }

    [Fact]
    public void SlopeAlignmentWithinClamp_PreservesAnimatedYaw()
    {
        var input = BaseInput(Up);
        input.MaxFootRotationRadians = 30f * (MathF.PI / 180f);
        Vector3 n = Vector3.Transform(Up, Quaternion.CreateFromAxisAngle(Vector3.UnitX, 20f * (MathF.PI / 180f)));

        Quaternion yawA = Quaternion.Identity;
        Quaternion yawB = Quaternion.CreateFromAxisAngle(Up, 40f * (MathF.PI / 180f));

        input.LeftFoot = Grounded(Vector3.Zero, yawA, Vector3.Zero, n);
        input.RightFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);
        var resultA = FootPlacementSolver.Solve(input);

        input.LeftFoot = Grounded(Vector3.Zero, yawB, Vector3.Zero, n);
        var resultB = FootPlacementSolver.Solve(input);

        Quaternion deltaA = resultA.LeftFoot.TargetRotation * Quaternion.Inverse(yawA);
        Quaternion deltaB = resultB.LeftFoot.TargetRotation * Quaternion.Inverse(yawB);

        float deltaAngle = TestHelpers.AngleBetweenDegrees(Quaternion.Identity, deltaA);
        Assert.True(MathF.Abs(deltaAngle - 20f) < 0.1f, $"delta angle {deltaAngle} != 20 deg");

        float deltaMismatch = TestHelpers.AngleBetweenDegrees(deltaA, deltaB);
        Assert.True(deltaMismatch < 0.1f, $"delta depends on animated yaw by {deltaMismatch} deg - yaw not preserved");

        Vector3 alignedUp = Vector3.Transform(Up, deltaA);
        float alignErr = TestHelpers.AngleBetweenDegrees(alignedUp, n);
        Assert.True(alignErr < 0.1f, $"delta did not align up with normal, off by {alignErr} deg");
    }

    [Fact]
    public void RotationClampAtMax()
    {
        var input = BaseInput(Up);
        input.MaxFootRotationRadians = 30f * (MathF.PI / 180f);
        Vector3 n = Vector3.Transform(Up, Quaternion.CreateFromAxisAngle(Vector3.UnitX, 50f * (MathF.PI / 180f)));
        input.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, Vector3.Zero, n);
        input.RightFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);

        var result = FootPlacementSolver.Solve(input);

        float angle = TestHelpers.AngleBetweenDegrees(Quaternion.Identity, result.LeftFoot.TargetRotation);
        Assert.True(MathF.Abs(angle - 30f) < 0.05f, $"clamp angle {angle} != 30 deg");

        Vector3 expectedAxis = Vector3.Normalize(Vector3.Cross(Up, n));
        Vector3 actualAxis = Vector3.Normalize(new Vector3(result.LeftFoot.TargetRotation.X, result.LeftFoot.TargetRotation.Y, result.LeftFoot.TargetRotation.Z));
        float axisErr = TestHelpers.AngleBetweenDegrees(expectedAxis, actualAxis);
        Assert.True(axisErr < 0.05f || MathF.Abs(axisErr - 180f) < 0.05f, $"axis mismatch {axisErr} deg");
    }

    [Fact]
    public void SteepSlope_Ungrounded()
    {
        var input = BaseInput(Up);
        input.MaxGroundSlopeRadians = 60f * (MathF.PI / 180f);
        Vector3 n = Vector3.Transform(Up, Quaternion.CreateFromAxisAngle(Vector3.UnitX, 70f * (MathF.PI / 180f)));
        input.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, Vector3.Zero, n);
        input.RightFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);

        var result = FootPlacementSolver.Solve(input);

        Assert.False(result.LeftFoot.Grounded);
        Assert.Equal(0f, result.LeftFoot.VerticalDelta);
    }

    [Fact]
    public void WeightSemantics_LinearInWeight_FeetUnweighted()
    {
        var baseline = BaseInput(Up);
        baseline.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, Up * -10f, Up);
        baseline.RightFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);

        var full = baseline;
        full.Weight = 1f;
        var fullResult = FootPlacementSolver.Solve(full);
        float expectedFull = fullResult.PelvisOffset;

        for (int i = 0; i <= 10; i++)
        {
            float w = i / 10f;
            var input = baseline;
            input.Weight = w;
            var result = FootPlacementSolver.Solve(input);

            Assert.Equal(expectedFull * w, result.PelvisOffset, 4);
            Assert.Equal(-10f, result.LeftFoot.VerticalDelta, 4);
            Assert.True(TestHelpers.AngleBetweenDegrees(result.LeftFoot.TargetRotation, fullResult.LeftFoot.TargetRotation) < 0.01f);
        }

        var zeroInput = baseline;
        zeroInput.Weight = 0f;
        Assert.Equal(0f, FootPlacementSolver.Solve(zeroInput).PelvisOffset);
    }

    [Fact]
    public void FootHeightOffset_AddsToDelta()
    {
        var input = BaseInput(Up);
        input.FootHeightOffset = 2f;
        input.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, Vector3.Zero, Up);
        input.RightFoot = Ungrounded(Vector3.Zero, Quaternion.Identity);

        var result = FootPlacementSolver.Solve(input);

        Assert.Equal(2f, result.LeftFoot.VerticalDelta, 4);
    }

    [Fact]
    public void DegenerateUp_FullPassthrough()
    {
        var input = BaseInput(Vector3.Zero);
        input.LeftFoot = Grounded(Vector3.Zero, Quaternion.Identity, Up * -10f, Up);
        input.RightFoot = Grounded(Vector3.Zero, Quaternion.Identity, Up * 10f, Up);

        var result = FootPlacementSolver.Solve(input);

        Assert.False(result.Solved);
        Assert.False(result.LeftFoot.Grounded);
        Assert.False(result.RightFoot.Grounded);
        Assert.Equal(0f, result.LeftFoot.VerticalDelta);
        Assert.Equal(0f, result.RightFoot.VerticalDelta);
        Assert.Equal(0f, result.PelvisOffset);
        Assert.Equal(input.LeftFoot.FootRotation, result.LeftFoot.TargetRotation);
        Assert.Equal(input.RightFoot.FootRotation, result.RightFoot.TargetRotation);
    }

    [Fact]
    public void SmoothOffset_RateZeroSnaps_DeltaZeroHolds_ConvergesMonotonically_FramerateIndependent()
    {
        Assert.Equal(5f, FootPlacementSolver.SmoothOffset(0f, 5f, 0f, 0.1f));
        Assert.Equal(3f, FootPlacementSolver.SmoothOffset(3f, 5f, 10f, 0f));

        float current = 0f;
        const float target = 10f;
        const float rate = 5f;
        float prevDistance = MathF.Abs(target - current);
        for (int i = 0; i < 60; i++)
        {
            current = FootPlacementSolver.SmoothOffset(current, target, rate, 1f / 60f);
            float distance = MathF.Abs(target - current);
            Assert.True(distance <= prevDistance + 1e-6f, $"step {i}: overshot or diverged");
            prevDistance = distance;
        }
        Assert.True(prevDistance < 0.5f, "did not converge toward target");

        float oneStep = FootPlacementSolver.SmoothOffset(0f, 10f, rate, 0.2f);
        float twoSteps = FootPlacementSolver.SmoothOffset(FootPlacementSolver.SmoothOffset(0f, 10f, rate, 0.1f), 10f, rate, 0.1f);
        Assert.True(MathF.Abs(oneStep - twoSteps) < 1e-4f, $"framerate dependence: {oneStep} vs {twoSteps}");
    }

    [Fact]
    public void SeededFuzz_NoNaNBoundedInvariantsHold()
    {
        var rng = new Random(54321);

        for (int i = 0; i < 500; i++)
        {
            var input = new FootPlacementInput
            {
                OriginPosition = new Vector3((float)(rng.NextDouble() * 20 - 10), (float)(rng.NextDouble() * 20 - 10), (float)(rng.NextDouble() * 20 - 10)),
                UpAxis = RandomUnitVector(rng),
                FootHeightOffset = (float)(rng.NextDouble() * 4 - 2),
                MaxStepUp = (float)rng.NextDouble() * 30f,
                MaxStepDown = (float)rng.NextDouble() * 30f,
                MaxPelvisDrop = (float)rng.NextDouble() * 30f,
                MaxPelvisRaise = (float)rng.NextDouble() * 10f,
                MaxFootRotationRadians = (float)rng.NextDouble() * MathF.PI,
                MaxGroundSlopeRadians = (float)rng.NextDouble() * MathF.PI,
                Weight = (float)rng.NextDouble(),
            };

            input.LeftFoot = RandomFoot(rng, input.OriginPosition);
            input.RightFoot = RandomFoot(rng, input.OriginPosition);

            var result = FootPlacementSolver.Solve(input);

            AssertFiniteFoot(result.LeftFoot, i, "left");
            AssertFiniteFoot(result.RightFoot, i, "right");
            Assert.False(float.IsNaN(result.PelvisOffset) || float.IsInfinity(result.PelvisOffset), $"iter {i}: pelvis not finite");

            if (result.LeftFoot.Grounded)
                Assert.InRange(result.LeftFoot.VerticalDelta, -input.MaxStepDown - 1e-3f, input.MaxStepUp + 1e-3f);
            if (result.RightFoot.Grounded)
                Assert.InRange(result.RightFoot.VerticalDelta, -input.MaxStepDown - 1e-3f, input.MaxStepUp + 1e-3f);

            Assert.InRange(result.PelvisOffset, -input.MaxPelvisDrop - 1e-3f, input.MaxPelvisRaise + 1e-3f);
        }
    }

    private static void AssertFiniteFoot(FootResult foot, int iter, string label)
    {
        var q = foot.TargetRotation;
        Assert.False(float.IsNaN(q.X) || float.IsNaN(q.Y) || float.IsNaN(q.Z) || float.IsNaN(q.W)
            || float.IsInfinity(q.X) || float.IsInfinity(q.Y) || float.IsInfinity(q.Z) || float.IsInfinity(q.W), $"iter {iter} {label}: rotation not finite");
        Assert.True(MathF.Abs(q.Length() - 1f) < 1e-3f, $"iter {iter} {label}: rotation not unit length ({q.Length()})");
        Assert.False(float.IsNaN(foot.VerticalDelta) || float.IsInfinity(foot.VerticalDelta), $"iter {iter} {label}: delta not finite");
    }

    private static FootInput RandomFoot(Random rng, Vector3 origin)
    {
        bool hasHit = rng.NextDouble() < 0.85;
        if (!hasHit)
            return Ungrounded(RandomPoint(rng), RandomRotation(rng));

        Vector3 hitPoint = origin + RandomUnitVector(rng) * (float)(rng.NextDouble() * 40);
        Vector3 normal = rng.NextDouble() < 0.05 ? Vector3.Zero : RandomUnitVector(rng);
        return Grounded(RandomPoint(rng), RandomRotation(rng), hitPoint, normal);
    }

    private static Vector3 RandomPoint(Random rng)
        => new((float)(rng.NextDouble() * 40 - 20), (float)(rng.NextDouble() * 40 - 20), (float)(rng.NextDouble() * 40 - 20));

    private static Quaternion RandomRotation(Random rng)
        => Quaternion.Normalize(new Quaternion(
            (float)(rng.NextDouble() * 2 - 1), (float)(rng.NextDouble() * 2 - 1),
            (float)(rng.NextDouble() * 2 - 1), (float)(rng.NextDouble() * 2 - 1)));

    private static Vector3 RandomUnitVector(Random rng)
    {
        while (true)
        {
            var v = new Vector3(
                (float)(rng.NextDouble() * 2 - 1),
                (float)(rng.NextDouble() * 2 - 1),
                (float)(rng.NextDouble() * 2 - 1));
            float lenSq = v.LengthSquared();
            if (lenSq > 1e-4f && lenSq < 1f)
                return v / MathF.Sqrt(lenSq);
        }
    }
}
