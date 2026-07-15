using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class LookAtTests
{
    // Canonical fixture per spec: bone at origin, identity rotation, LocalAimDirection = (0,0,1),
    // MaxAngleRadians = 45 degrees, Weight = 1 unless stated otherwise.
    private static readonly Vector3 CanonicalAim = Vector3.UnitZ;
    private const float CanonicalMaxAngleDeg = 45f;

    private static LookAtInput Canonical(Vector3 targetPosition, float maxAngleDeg = CanonicalMaxAngleDeg, float weight = 1f)
    {
        return new LookAtInput
        {
            BonePosition = Vector3.Zero,
            BoneRotation = Quaternion.Identity,
            LocalAimDirection = CanonicalAim,
            TargetPosition = targetPosition,
            MaxAngleRadians = maxAngleDeg * (MathF.PI / 180f),
            Weight = weight,
        };
    }

    private static Vector3 FinalAimDirection(LookAtResult result)
        => Vector3.Normalize(Vector3.Transform(CanonicalAim, result.BoneRotation));

    private static void AssertFiniteRotation(Quaternion q)
    {
        Assert.False(float.IsNaN(q.X) || float.IsNaN(q.Y) || float.IsNaN(q.Z) || float.IsNaN(q.W)
            || float.IsInfinity(q.X) || float.IsInfinity(q.Y) || float.IsInfinity(q.Z) || float.IsInfinity(q.W));
        Assert.True(MathF.Abs(q.Length() - 1f) < 1e-4f, $"result rotation not unit length: {q.Length()}");
    }

    [Fact]
    public void WithinCone_ReachesTargetExactly()
    {
        var input = Canonical(new Vector3(0.3f, 0f, 1f));
        var result = LookAtSolver.Solve(input);

        Assert.True(result.Solved);
        float err = TestHelpers.AngleBetweenDegrees(FinalAimDirection(result), Vector3.Normalize(input.TargetPosition));
        Assert.True(err < 1e-4f * (180f / MathF.PI), $"within-cone error {err} deg");
    }

    [Fact]
    public void ExactlyAtClampBoundary_NoDistortion()
    {
        Vector3 targetDir = Vector3.Transform(CanonicalAim, Quaternion.CreateFromAxisAngle(Vector3.UnitY, CanonicalMaxAngleDeg * (MathF.PI / 180f)));
        var input = Canonical(targetDir * 2f);
        var result = LookAtSolver.Solve(input);

        float err = TestHelpers.AngleBetweenDegrees(FinalAimDirection(result), targetDir);
        Assert.True(err < 0.01f, $"boundary error {err} deg");
    }

    [Fact]
    public void BeyondClamp_AngleEqualsMax_StaysInPlane()
    {
        Vector3 targetDir = Vector3.Transform(CanonicalAim, Quaternion.CreateFromAxisAngle(Vector3.UnitY, 90f * (MathF.PI / 180f)));
        var input = Canonical(targetDir * 2f);
        var result = LookAtSolver.Solve(input);

        Vector3 finalDir = FinalAimDirection(result);
        float angleFromAnimated = TestHelpers.AngleBetweenDegrees(finalDir, CanonicalAim);
        Assert.True(MathF.Abs(angleFromAnimated - CanonicalMaxAngleDeg) < 0.01f, $"clamp angle {angleFromAnimated} deg");

        Vector3 planeNormal = Vector3.Cross(CanonicalAim, targetDir);
        float offPlane = Vector3.Dot(finalDir, Vector3.Normalize(planeNormal));
        Assert.True(MathF.Abs(offPlane) < 1e-4f, $"final direction off bend plane by {offPlane}");
    }

    [Fact]
    public void OrbitAt60Degrees_NoFlip_360MatchesZero()
    {
        const float orbitAngleDeg = 60f;
        Quaternion prev = Quaternion.Identity;
        Quaternion first = Quaternion.Identity;
        Quaternion last = Quaternion.Identity;

        for (int i = 0; i <= 360; i++)
        {
            Vector3 targetDir = Vector3.Transform(
                Vector3.Transform(CanonicalAim, Quaternion.CreateFromAxisAngle(Vector3.UnitX, orbitAngleDeg * (MathF.PI / 180f))),
                Quaternion.CreateFromAxisAngle(Vector3.UnitZ, i * (MathF.PI / 180f)));

            var input = Canonical(targetDir * 2f);
            var result = LookAtSolver.Solve(input);
            AssertFiniteRotation(result.BoneRotation);

            if (i == 0) first = result.BoneRotation;
            if (i == 360) last = result.BoneRotation;

            if (i > 0)
            {
                float delta = TestHelpers.AngleBetweenDegrees(prev, result.BoneRotation);
                Assert.True(delta < 5f, $"orbit step {i}: adjacent rotation jumped {delta} deg");
            }
            prev = result.BoneRotation;
        }

        float wrap = TestHelpers.AngleBetweenDegrees(first, last);
        Assert.True(wrap < 0.01f, $"orbit did not close: {wrap} deg between sample 0 and 360");
    }

    [Fact]
    public void WeightSweep_NoPopping_EndpointsExact()
    {
        Vector3 targetDir = Vector3.Transform(CanonicalAim, Quaternion.CreateFromAxisAngle(Vector3.UnitY, 90f * (MathF.PI / 180f)));
        var full = Canonical(targetDir * 2f, weight: 1f);
        var fullResult = LookAtSolver.Solve(full);

        Quaternion prev = Quaternion.Identity;
        for (int i = 0; i <= 100; i++)
        {
            float w = i / 100f;
            var input = Canonical(targetDir * 2f, weight: w);
            var result = LookAtSolver.Solve(input);

            if (i == 0)
            {
                Assert.True(TestHelpers.AngleBetweenDegrees(result.BoneRotation, input.BoneRotation) < 1e-3f);
            }
            if (i == 100)
            {
                Assert.True(TestHelpers.AngleBetweenDegrees(result.BoneRotation, fullResult.BoneRotation) < 1e-3f);
            }
            if (i > 0)
            {
                float delta = TestHelpers.AngleBetweenDegrees(prev, result.BoneRotation);
                Assert.True(delta < 2f, $"weight step {w}: popped by {delta} deg");
            }
            prev = result.BoneRotation;
        }
    }

    [Fact]
    public void ZeroDistanceTarget_PassthroughNotSolved()
    {
        var input = Canonical(Vector3.Zero);
        var result = LookAtSolver.Solve(input);

        Assert.False(result.Solved);
        Assert.Equal(input.BoneRotation, result.BoneRotation);
        AssertFiniteRotation(result.BoneRotation);
    }

    [Fact]
    public void MaxAngleZero_OutputEqualsInputRegardlessOfTarget()
    {
        var input = Canonical(new Vector3(5f, 3f, -2f), maxAngleDeg: 0f);
        var result = LookAtSolver.Solve(input);

        Assert.Equal(input.BoneRotation.X, result.BoneRotation.X, 5);
        Assert.Equal(input.BoneRotation.Y, result.BoneRotation.Y, 5);
        Assert.Equal(input.BoneRotation.Z, result.BoneRotation.Z, 5);
        Assert.Equal(input.BoneRotation.W, result.BoneRotation.W, 5);
    }

    [Fact]
    public void MaxAngleAt180_Unclamped()
    {
        Vector3 targetDir = Vector3.Transform(CanonicalAim, Quaternion.CreateFromAxisAngle(Vector3.UnitY, 170f * (MathF.PI / 180f)));
        var input = Canonical(targetDir * 2f, maxAngleDeg: 180f);
        var result = LookAtSolver.Solve(input);

        float err = TestHelpers.AngleBetweenDegrees(FinalAimDirection(result), targetDir);
        Assert.True(err < 1e-4f * (180f / MathF.PI), $"unclamped error {err} deg");
    }

    [Fact]
    public void AntiparallelDegenerate_NoNaN_Deterministic()
    {
        // MaxAngleRadians deliberately exceeds pi (the solver does not clamp its own input range;
        // only the component's [Range(0,180)] Property restricts the editor-facing value) so the
        // clamp never engages here.
        var input = Canonical(-CanonicalAim * 2f, maxAngleDeg: 0f);
        input.MaxAngleRadians = 3.5f;

        var result1 = LookAtSolver.Solve(input);
        var result2 = LookAtSolver.Solve(input);

        AssertFiniteRotation(result1.BoneRotation);
        Assert.Equal(result1.BoneRotation.X, result2.BoneRotation.X);
        Assert.Equal(result1.BoneRotation.Y, result2.BoneRotation.Y);
        Assert.Equal(result1.BoneRotation.Z, result2.BoneRotation.Z);
        Assert.Equal(result1.BoneRotation.W, result2.BoneRotation.W);
    }

    [Fact]
    public void AnalyzeBindPose_RealChild_ReliableAimDirection()
    {
        var bind = LookAtSolver.AnalyzeBindPose(Vector3.Zero, Quaternion.Identity, new Vector3(0f, 0f, 2f));

        Assert.True(bind.IsReliable);
        float err = TestHelpers.AngleBetweenDegrees(bind.LocalAimDirection, Vector3.UnitZ);
        Assert.True(err < 0.01f, $"bind aim direction off by {err} deg");
    }

    [Fact]
    public void AnalyzeBindPose_NoChildOrDegenerateChild_FallsBackDeterministically()
    {
        var noChild = LookAtSolver.AnalyzeBindPose(Vector3.Zero, Quaternion.Identity, null);
        Assert.False(noChild.IsReliable);
        Assert.Equal(Vector3.UnitX, noChild.LocalAimDirection);

        var degenerateChild = LookAtSolver.AnalyzeBindPose(Vector3.Zero, Quaternion.Identity, Vector3.Zero);
        Assert.False(degenerateChild.IsReliable);
        Assert.Equal(Vector3.UnitX, degenerateChild.LocalAimDirection);

        var noChildAgain = LookAtSolver.AnalyzeBindPose(Vector3.Zero, Quaternion.Identity, null);
        Assert.Equal(noChild.LocalAimDirection, noChildAgain.LocalAimDirection);
    }

    [Fact]
    public void Purity_SameInputSolvedTwice_BitwiseIdentical()
    {
        var input = Canonical(new Vector3(0.7f, -0.4f, 1.3f));
        var result1 = LookAtSolver.Solve(input);
        var result2 = LookAtSolver.Solve(input);

        Assert.Equal(result1.BoneRotation.X, result2.BoneRotation.X);
        Assert.Equal(result1.BoneRotation.Y, result2.BoneRotation.Y);
        Assert.Equal(result1.BoneRotation.Z, result2.BoneRotation.Z);
        Assert.Equal(result1.BoneRotation.W, result2.BoneRotation.W);
        Assert.Equal(result1.Solved, result2.Solved);
    }
}
