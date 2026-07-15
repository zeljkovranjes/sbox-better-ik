using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class OrbitTests
{
    [Fact]
    public void ThreeSixtyOrbit_ContinuityNoFlip()
    {
        var baseInput = TestHelpers.CanonicalInput();
        baseInput.HasPole = true;
        baseInput.PoleHint = new Vector3(0, 10, 0);

        float lmax = TestHelpers.Lmax(baseInput);

        TwoBoneIkResult? prev = null;
        Vector3? prevMid = null;
        TwoBoneIkResult first = default;
        Vector3 firstMid = default;

        for (int deg = 0; deg <= 360; deg++)
        {
            float theta = deg * MathF.PI / 180f;
            var input = baseInput;
            input.TargetPosition = input.RootPosition + 1.2f * new Vector3(MathF.Cos(theta), 0.25f, MathF.Sin(theta));

            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            Vector3 aHat = Vector3.Normalize(input.TargetPosition - input.RootPosition);
            Vector3 polePerp = TestHelpers.ProjectPerpendicular(input.PoleHint, aHat);
            float elbowSide = Vector3.Dot(result.MidPosition - input.RootPosition, Vector3.Normalize(polePerp));
            Assert.True(elbowSide > 0f, $"deg {deg}: elbow not on pole side");

            if (prev.HasValue)
            {
                float rootDelta = TestHelpers.AngleBetweenDegrees(prev.Value.RootRotation, result.RootRotation);
                float midDelta = TestHelpers.AngleBetweenDegrees(prev.Value.MidRotation, result.MidRotation);
                float endDelta = TestHelpers.AngleBetweenDegrees(prev.Value.EndRotation, result.EndRotation);
                Assert.True(rootDelta < 5f, $"deg {deg}: root rotation jump {rootDelta}");
                Assert.True(midDelta < 5f, $"deg {deg}: mid rotation jump {midDelta}");
                Assert.True(endDelta < 5f, $"deg {deg}: end rotation jump {endDelta}");

                float midMove = (result.MidPosition - prevMid!.Value).Length();
                Assert.True(midMove < 0.1f, $"deg {deg}: mid position jump {midMove}");
            }
            else
            {
                first = result;
                firstMid = result.MidPosition;
            }

            prev = result;
            prevMid = result.MidPosition;
        }

        Assert.True((prev!.Value.MidPosition - firstMid).Length() < 1e-4f, "360 sample does not match 0 sample position");
        Assert.True(TestHelpers.AngleBetweenDegrees(prev.Value.RootRotation, first.RootRotation) < 0.01f, "360 sample does not match 0 sample rotation");
        _ = lmax;
    }

    [Fact]
    public void PoleOrbit()
    {
        var baseInput = TestHelpers.CanonicalInput();
        baseInput.TargetPosition = new Vector3(1.6f, 0f, 0.2f);
        baseInput.HasPole = true;

        Vector3 aHat = Vector3.Normalize(baseInput.TargetPosition - baseInput.RootPosition);
        Vector3 seedPole = TestHelpers.ProjectPerpendicular(new Vector3(0, 0, 1), aHat);
        seedPole = Vector3.Normalize(seedPole);

        TwoBoneIkResult? prev = null;

        for (int deg = 0; deg <= 360; deg++)
        {
            float theta = deg * MathF.PI / 180f;
            var input = baseInput;
            input.PoleHint = Vector3.Transform(seedPole, Quaternion.CreateFromAxisAngle(aHat, theta));

            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            Vector3 expectedElbowDir = Vector3.Normalize(TestHelpers.ProjectPerpendicular(input.PoleHint, aHat));
            Vector3 actualElbowDir = Vector3.Normalize(TestHelpers.ProjectPerpendicular(result.MidPosition - input.RootPosition, aHat));
            float dot = Math.Clamp(Vector3.Dot(expectedElbowDir, actualElbowDir), -1f, 1f);
            float angleErr = MathF.Acos(dot) * 180f / MathF.PI;
            Assert.True(angleErr < 0.5f, $"deg {deg}: elbow direction off by {angleErr} degrees");

            Assert.True((result.EndPosition - input.TargetPosition).Length() < 1e-4f, $"deg {deg}: end off target");

            if (prev.HasValue)
            {
                float rootDelta = TestHelpers.AngleBetweenDegrees(prev.Value.RootRotation, result.RootRotation);
                Assert.True(rootDelta < 5f, $"deg {deg}: root rotation jump {rootDelta}");
            }

            prev = result;
        }
    }

    [Fact]
    public void PoleAngleOffsetSweep()
    {
        var baseInput = TestHelpers.CanonicalInput();
        baseInput.TargetPosition = new Vector3(1.6f, 0f, 0.2f);
        baseInput.HasPole = true;
        baseInput.PoleHint = new Vector3(0, 0, 1);

        Vector3 aHat = Vector3.Normalize(baseInput.TargetPosition - baseInput.RootPosition);
        Vector3 pPerp = Vector3.Normalize(TestHelpers.ProjectPerpendicular(baseInput.PoleHint, aHat));

        // Fixed offset check at pi/2.
        {
            var input = baseInput;
            input.PoleAngleOffsetRadians = MathF.PI / 2f;
            var result = TwoBoneIkSolver.Solve(input);

            Vector3 expected = Vector3.Transform(pPerp, Quaternion.CreateFromAxisAngle(aHat, MathF.PI / 2f));
            Vector3 actual = Vector3.Normalize(TestHelpers.ProjectPerpendicular(result.MidPosition - input.RootPosition, aHat));
            float dot = Math.Clamp(Vector3.Dot(expected, actual), -1f, 1f);
            float angleErr = MathF.Acos(dot) * 180f / MathF.PI;
            Assert.True(angleErr < 0.5f, $"offset pi/2: elbow direction off by {angleErr} degrees");
        }

        TwoBoneIkResult? prev = null;
        TwoBoneIkResult first = default;

        for (int deg = 0; deg <= 360; deg++)
        {
            float offset = deg * MathF.PI / 180f;
            var input = baseInput;
            input.PoleAngleOffsetRadians = offset;
            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            if (deg == 0) first = result;

            if (prev.HasValue)
            {
                float rootDelta = TestHelpers.AngleBetweenDegrees(prev.Value.RootRotation, result.RootRotation);
                Assert.True(rootDelta < 5f, $"offset deg {deg}: root rotation jump {rootDelta}");
            }

            prev = result;
        }

        Assert.True(TestHelpers.AngleBetweenDegrees(prev!.Value.RootRotation, first.RootRotation) < 0.01f, "offset 2pi does not match offset 0");
    }
}
