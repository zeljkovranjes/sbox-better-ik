using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class FabrikTests
{
    // Canonical fixture: 5 joints along +X, spacing 1.0, identity rotations. Total length = 4.
    private static Vector3[] CanonicalPositions()
        => new[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(2, 0, 0), new Vector3(3, 0, 0), new Vector3(4, 0, 0) };

    private static Quaternion[] CanonicalRotations(int n)
    {
        var r = new Quaternion[n];
        for (int i = 0; i < n; i++) r[i] = Quaternion.Identity;
        return r;
    }

    private static FabrikInput CanonicalInput(Vector3 target, float weight = 1f, int maxIterations = 16, float tolerance = 1e-4f)
        => new()
        {
            JointPositions = CanonicalPositions(),
            TargetPosition = target,
            Weight = weight,
            MaxIterations = maxIterations,
            Tolerance = tolerance,
        };

    private static void AssertFinite(Vector3[] positions)
    {
        foreach (var p in positions)
            Assert.False(float.IsNaN(p.X) || float.IsNaN(p.Y) || float.IsNaN(p.Z)
                || float.IsInfinity(p.X) || float.IsInfinity(p.Y) || float.IsInfinity(p.Z), $"position not finite: {p}");
    }

    [Fact]
    public void ReachableTarget_ConvergesWithinTolerance()
    {
        var input = CanonicalInput(new Vector3(2, 1, 0));
        var result = FabrikSolver.Solve(input);

        AssertFinite(result.JointPositions);
        Assert.True(result.Converged);
        float endErr = (result.JointPositions[^1] - input.TargetPosition).Length();
        Assert.True(endErr <= input.Tolerance, $"end error {endErr} exceeds tolerance");
    }

    [Fact]
    public void UnreachableTarget_ProducesExactStraightChain()
    {
        var input = CanonicalInput(new Vector3(20, 0, 0));
        var result = FabrikSolver.Solve(input);

        AssertFinite(result.JointPositions);
        Assert.False(result.Converged);
        Assert.Equal(0, result.IterationsUsed);

        float totalLength = 4f;
        float endDist = (result.JointPositions[^1] - input.JointPositions[0]).Length();
        Assert.True(MathF.Abs(endDist - totalLength) < 1e-4f, $"end not at total length: {endDist}");

        // All joints collinear along the root-to-target ray.
        Vector3 dir = Vector3.Normalize(input.TargetPosition - input.JointPositions[0]);
        for (int i = 1; i < result.JointPositions.Length; i++)
        {
            Vector3 fromRoot = result.JointPositions[i] - result.JointPositions[0];
            Vector3 perp = fromRoot - Vector3.Dot(fromRoot, dir) * dir;
            Assert.True(perp.Length() < 1e-4f, $"joint {i} off the ray by {perp.Length()}");
        }
    }

    [Fact]
    public void SegmentLengthsPreserved_AtFullWeight()
    {
        var input = CanonicalInput(new Vector3(1.5f, 2f, -1f));
        var result = FabrikSolver.Solve(input);

        for (int i = 0; i < result.JointPositions.Length - 1; i++)
        {
            float solvedLen = (result.JointPositions[i + 1] - result.JointPositions[i]).Length();
            Assert.True(MathF.Abs(solvedLen - 1f) < 1e-3f, $"segment {i} length drifted to {solvedLen}");
        }
    }

    [Fact]
    public void RootPosition_InvariantAcrossReachableAndUnreachable()
    {
        var reachable = FabrikSolver.Solve(CanonicalInput(new Vector3(2, 1, 0)));
        var unreachable = FabrikSolver.Solve(CanonicalInput(new Vector3(20, 0, 0)));

        Assert.Equal(Vector3.Zero, reachable.JointPositions[0]);
        Assert.Equal(Vector3.Zero, unreachable.JointPositions[0]);
    }

    [Fact]
    public void WeightZero_ExactPassthrough()
    {
        var input = CanonicalInput(new Vector3(2, 1, 0), weight: 0f);
        var result = FabrikSolver.Solve(input);

        Assert.Equal(0, result.IterationsUsed);
        Assert.False(result.Converged);
        for (int i = 0; i < input.JointPositions.Length; i++)
            Assert.Equal(input.JointPositions[i], result.JointPositions[i]);
    }

    [Fact]
    public void WeightHalf_BetweenAnimatedAndSolved()
    {
        var target = new Vector3(2, 1, 0);
        var full = FabrikSolver.Solve(CanonicalInput(target, weight: 1f));
        var half = FabrikSolver.Solve(CanonicalInput(target, weight: 0.5f));
        var animated = CanonicalPositions();

        for (int i = 0; i < animated.Length; i++)
        {
            Vector3 expected = Vector3.Lerp(animated[i], full.JointPositions[i], 0.5f);
            Assert.True((half.JointPositions[i] - expected).Length() < 1e-4f, $"joint {i} not the expected lerp");
        }
    }

    [Fact]
    public void Determinism_SameInputTwice_Identical()
    {
        var input = CanonicalInput(new Vector3(2, 1, 0));
        var r1 = FabrikSolver.Solve(input);
        var r2 = FabrikSolver.Solve(input);

        for (int i = 0; i < r1.JointPositions.Length; i++)
            Assert.Equal(r1.JointPositions[i], r2.JointPositions[i]);
        Assert.Equal(r1.IterationsUsed, r2.IterationsUsed);
        Assert.Equal(r1.Converged, r2.Converged);
    }

    [Fact]
    public void TargetAtRoot_NoNaN_Passthrough()
    {
        var input = CanonicalInput(Vector3.Zero);
        var result = FabrikSolver.Solve(input);

        AssertFinite(result.JointPositions);
        Assert.False(result.Converged);
        for (int i = 0; i < input.JointPositions.Length; i++)
            Assert.Equal(input.JointPositions[i], result.JointPositions[i]);
    }

    [Fact]
    public void ZeroLengthSegment_NoNaN()
    {
        var positions = CanonicalPositions();
        positions[2] = positions[1]; // collapse one segment to zero length
        var input = new FabrikInput
        {
            JointPositions = positions,
            TargetPosition = new Vector3(2, 1, 0),
            Weight = 1f,
            MaxIterations = 16,
            Tolerance = 1e-4f,
        };

        var result = FabrikSolver.Solve(input);
        AssertFinite(result.JointPositions);
    }

    [Fact]
    public void TwoJointChain_PointsAtTargetAtFixedLength()
    {
        var positions = new[] { new Vector3(0, 0, 0), new Vector3(5, 0, 0) };
        var input = new FabrikInput
        {
            JointPositions = positions,
            TargetPosition = new Vector3(2, 3, 0), // closer than segment length 5, not on the segment's own axis
            Weight = 1f,
            MaxIterations = 16,
            Tolerance = 1e-5f,
        };

        var result = FabrikSolver.Solve(input);
        AssertFinite(result.JointPositions);

        Assert.Equal(Vector3.Zero, result.JointPositions[0]);
        float tipDist = (result.JointPositions[1] - result.JointPositions[0]).Length();
        Assert.True(MathF.Abs(tipDist - 5f) < 1e-3f, $"tip not at fixed segment length: {tipDist}");

        Vector3 expectedDir = Vector3.Normalize(input.TargetPosition);
        Vector3 actualDir = Vector3.Normalize(result.JointPositions[1]);
        Assert.True((expectedDir - actualDir).Length() < 1e-3f, "tip does not point toward target");
    }

    [Fact]
    public void IterationCap_RespectedAndBounded()
    {
        var input = CanonicalInput(new Vector3(1.5f, 2f, -3f), maxIterations: 1, tolerance: 1e-9f);
        var result = FabrikSolver.Solve(input);

        AssertFinite(result.JointPositions);
        Assert.Equal(1, result.IterationsUsed);
    }

    [Fact]
    public void DeriveRotations_IdentityWhenSolvedMatchesAnimated()
    {
        var positions = CanonicalPositions();
        var rotations = CanonicalRotations(positions.Length);
        rotations[1] = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.3f);

        var result = FabrikSolver.DeriveRotations(positions, positions, rotations);

        for (int i = 0; i < rotations.Length; i++)
        {
            float dot = MathF.Abs(Quaternion.Dot(Quaternion.Normalize(result[i]), rotations[i]));
            Assert.True(dot > 1f - 1e-4f, $"joint {i} rotation changed when solved pose matched animated");
        }
    }

    [Fact]
    public void DeriveRotations_KnownNinetyDegreeCase()
    {
        var animated = new[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0) };
        var solved = new[] { new Vector3(0, 0, 0), new Vector3(0, 1, 0) };
        var rotations = new[] { Quaternion.Identity, Quaternion.Identity };

        var result = FabrikSolver.DeriveRotations(animated, solved, rotations);

        Vector3 rotatedX = Vector3.Transform(Vector3.UnitX, result[0]);
        Assert.True((rotatedX - Vector3.UnitY).Length() < 1e-4f, $"expected delta to rotate +X to +Y, got {rotatedX}");

        float angle = 2f * MathF.Acos(Math.Clamp(MathF.Abs(result[0].W), -1f, 1f)) * (180f / MathF.PI);
        Assert.True(MathF.Abs(angle - 90f) < 0.05f, $"angle {angle} != 90 degrees");
    }

    [Fact]
    public void DeriveRotations_LeafBoneUsesOwnAnimatedRotation()
    {
        // Last segment's direction change (perpendicular, not antiparallel) is deliberately chosen
        // so this test's own inline shortest-arc math matches IkMath.FromToRotation's non-degenerate
        // branch exactly, without needing that internal method's antiparallel fallback.
        var animated = new[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(2, 0, 0) };
        var solved = new[] { new Vector3(0, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 1, 1) };
        var midRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.5f);
        var leafRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 1.2f);
        var rotations = new[] { Quaternion.Identity, midRotation, leafRotation };

        var result = FabrikSolver.DeriveRotations(animated, solved, rotations);

        // Leaf delta must be the LAST real segment's delta (joint 1 -> 2), composed onto the
        // leaf's OWN animated rotation, not joint 1's. Shortest-arc delta computed inline here
        // (not via the internal IkMath.FromToRotation, which isn't visible across assemblies)
        // since this segment pair is not antiparallel and needs no degenerate-case fallback.
        Vector3 lastAnimatedDir = Vector3.Normalize(animated[2] - animated[1]);
        Vector3 lastSolvedDir = Vector3.Normalize(solved[2] - solved[1]);
        float cosAngle = Math.Clamp(Vector3.Dot(lastAnimatedDir, lastSolvedDir), -1f, 1f);
        Vector3 axis = Vector3.Normalize(Vector3.Cross(lastAnimatedDir, lastSolvedDir));
        var expectedDelta = Quaternion.CreateFromAxisAngle(axis, MathF.Acos(cosAngle));
        var expectedLeaf = Quaternion.Normalize(expectedDelta * leafRotation);

        float dot = MathF.Abs(Quaternion.Dot(Quaternion.Normalize(result[2]), expectedLeaf));
        Assert.True(dot > 1f - 1e-4f, "leaf bone rotation did not use its own animated rotation as the base");

        // And it must differ from naively reusing joint 1's result composed the same way.
        Assert.NotEqual(result[1].X, result[2].X);
    }
}
