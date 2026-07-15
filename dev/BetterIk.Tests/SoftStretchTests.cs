using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class SoftStretchTests
{
    private static float SoftFormula(float d, float lmax, float softFraction)
    {
        float dSoft = (1f - softFraction) * lmax;
        float r = softFraction * lmax;
        return d <= dSoft ? d : dSoft + r * (1f - MathF.Exp(-(d - dSoft) / r));
    }

    [Fact]
    public void SoftClamp_MatchesFormula_MonotonicAndNeverStraight()
    {
        var baseInput = TestHelpers.CanonicalInput();
        baseInput.HasPole = true;
        baseInput.PoleHint = new Vector3(0, 0, 1);
        baseInput.SoftFraction = 0.1f;

        float lmax = TestHelpers.Lmax(baseInput);
        float dSoft = 0.9f * lmax;

        float prevDist = float.NegativeInfinity;
        float? slopeBelow = null;
        float? slopeAbove = null;

        int steps = 200;
        for (int i = 0; i <= steps; i++)
        {
            float d = (0.5f + 1.5f * (i / (float)steps)) * lmax; // 0.5*lmax .. 2*lmax
            var input = baseInput;
            input.TargetPosition = input.RootPosition + new Vector3(1, 0, 0) * d;

            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            float dist = (result.EndPosition - input.RootPosition).Length();
            float expected = SoftFormula(d, lmax, 0.1f);

            Assert.True(MathF.Abs(dist - expected) < 1e-5f * lmax, $"d={d}: dist {dist} vs formula {expected}");

            if (d <= dSoft)
                Assert.True(MathF.Abs(dist - d) < 1e-4f, $"d={d}: expected exact solve below dSoft");

            Assert.True(dist > prevDist, $"d={d}: not monotonic, dist {dist} <= prev {prevDist}");
            prevDist = dist;

            Assert.True(dist < lmax - 1e-6f, $"d={d}: reached full extension {dist} >= {lmax}");

            // Elbow angle strictly > 0 (chain never perfectly straight): mid must be off the root-target line.
            Vector3 aHat = Vector3.Normalize(input.TargetPosition - input.RootPosition);
            float offAxis = TestHelpers.ProjectPerpendicular(result.MidPosition - input.RootPosition, aHat).Length();
            Assert.True(offAxis > 1e-6f * lmax, $"d={d}: chain went straight, offAxis={offAxis}");

            // Track finite-difference slope just below/above dSoft for a C1 continuity check.
            if (MathF.Abs(d - dSoft) < 0.02f * lmax)
            {
                float dPrev = d - 0.001f * lmax;
                float dNext = d + 0.001f * lmax;
                float slope = (SoftFormula(dNext, lmax, 0.1f) - SoftFormula(dPrev, lmax, 0.1f)) / (0.002f * lmax);
                if (d < dSoft) slopeBelow = slope;
                else slopeAbove ??= slope;
            }
        }

        Assert.True(slopeBelow.HasValue && slopeAbove.HasValue, "did not sample near dSoft");
        Assert.True(MathF.Abs(slopeBelow!.Value - slopeAbove!.Value) < 0.05f * MathF.Max(MathF.Abs(slopeBelow.Value), 1f),
            $"slope discontinuity at dSoft: {slopeBelow} vs {slopeAbove}");
    }

    [Fact]
    public void Stretch_AppliesUniformScale_CappedByMaxStretch()
    {
        var baseInput = TestHelpers.CanonicalInput();
        baseInput.HasPole = true;
        baseInput.PoleHint = new Vector3(0, 0, 1);
        baseInput.MaxStretch = 0.2f;
        baseInput.SoftFraction = 0f;

        float lmax = TestHelpers.Lmax(baseInput);
        float l1 = TestHelpers.L1(baseInput);
        float l2 = TestHelpers.L2(baseInput);

        // 1.1x reach: fully stretchable within the 0.2 cap.
        {
            var input = baseInput;
            input.TargetPosition = input.RootPosition + new Vector3(1, 0, 0) * (1.1f * lmax);
            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            Assert.True((result.EndPosition - input.TargetPosition).Length() < 1e-4f, "1.1x reach: end not on target");
            Assert.True(MathF.Abs((result.MidPosition - input.RootPosition).Length() - 1.1f * l1) < 1e-4f);
            Assert.True(MathF.Abs((result.EndPosition - result.MidPosition).Length() - 1.1f * l2) < 1e-4f);
            Assert.True(MathF.Abs(result.AppliedStretch - 1.1f) < 1e-4f, $"AppliedStretch {result.AppliedStretch}");
        }

        // 1.5x reach: capped at 1.2x.
        {
            var input = baseInput;
            input.TargetPosition = input.RootPosition + new Vector3(1, 0, 0) * (1.5f * lmax);
            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            var expectedEnd = input.RootPosition + new Vector3(1, 0, 0) * (1.2f * lmax);
            Assert.True((result.EndPosition - expectedEnd).Length() < 1e-4f, $"1.5x reach: end {result.EndPosition} vs {expectedEnd}");
            Assert.True(MathF.Abs(result.AppliedStretch - 1.2f) < 1e-4f, $"AppliedStretch {result.AppliedStretch}");
        }

        // Half weight at 1.1x reach: stretch blends too.
        {
            var input = baseInput;
            input.TargetPosition = input.RootPosition + new Vector3(1, 0, 0) * (1.1f * lmax);
            input.MasterWeight = 0.5f;
            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            float expectedStretch = 1f + (1.1f - 1f) * 0.5f;
            Assert.True(MathF.Abs(result.AppliedStretch - expectedStretch) < 1e-4f, $"AppliedStretch {result.AppliedStretch} vs {expectedStretch}");
            float midLen = (result.MidPosition - input.RootPosition).Length();
            Assert.True(MathF.Abs(midLen - expectedStretch * l1) < 1e-4f, $"mid length {midLen} vs {expectedStretch * l1}");
        }
    }
}
