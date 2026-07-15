using System.Numerics;
using BetterIk.Maths;
using Xunit;

namespace BetterIk.Tests;

public class FuzzTests
{
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

    [Fact]
    public void SeededFuzz_1000Iterations_NoNaNBoundedInvariantsHold()
    {
        var rng = new Random(12345);

        for (int i = 0; i < 1000; i++)
        {
            float l1 = 0.1f + (float)rng.NextDouble() * 1.9f;
            float l2 = 0.1f + (float)rng.NextDouble() * 1.9f;
            float lmax = l1 + l2;

            Vector3 root = Vector3.Zero;
            Vector3 dir1 = RandomUnitVector(rng);
            Vector3 mid = root + dir1 * l1;

            // Perturb dir2 away from dir1 so the pose isn't perfectly collinear (fuzz targets
            // generic rig geometry, collinear poses are covered by dedicated tests).
            Vector3 dir2 = Vector3.Normalize(dir1 + RandomUnitVector(rng) * 0.7f);
            Vector3 end = mid + dir2 * l2;

            float targetDist = (0.1f + (float)rng.NextDouble() * 1.4f) * lmax;
            Vector3 targetDir = RandomUnitVector(rng);
            Vector3 target = root + targetDir * targetDist;

            Vector3 aHatApprox = Vector3.Normalize(target - root);
            Vector3 poleCandidate = RandomUnitVector(rng);
            Vector3 polePerp = poleCandidate - Vector3.Dot(poleCandidate, aHatApprox) * aHatApprox;
            if (polePerp.Length() < 0.05f)
                polePerp = Vector3.UnitY - Vector3.Dot(Vector3.UnitY, aHatApprox) * aHatApprox;

            var bind = TwoBoneIkSolver.AnalyzeBindPose(root, mid, end);

            var input = new TwoBoneIkInput
            {
                RootPosition = root,
                MidPosition = mid,
                EndPosition = end,
                RootRotation = Quaternion.Identity,
                MidRotation = Quaternion.Identity,
                EndRotation = Quaternion.Identity,
                TargetPosition = target,
                TargetRotation = Quaternion.Identity,
                HasPole = true,
                PoleHint = polePerp,
                PoleAngleOffsetRadians = 0f,
                FallbackBendNormal = bind.BendNormal,
                PositionWeight = (float)rng.NextDouble(),
                RotationWeight = (float)rng.NextDouble(),
                MasterWeight = (float)rng.NextDouble(),
                SoftFraction = rng.NextDouble() < 0.5 ? 0f : (float)rng.NextDouble() * 0.4f,
                MaxStretch = rng.NextDouble() < 0.5 ? 0f : (float)rng.NextDouble() * 0.5f,
            };

            var result = TwoBoneIkSolver.Solve(input);
            TestHelpers.AssertFinite(result);

            if (input.MaxStretch < 1e-6f)
            {
                float midLen = (result.MidPosition - root).Length();
                float endLen = (result.EndPosition - result.MidPosition).Length();
                Assert.True(MathF.Abs(midLen - l1) < 1e-4f * lmax, $"iter {i}: mid length drifted, {midLen} vs {l1}");
                Assert.True(MathF.Abs(endLen - l2) < 1e-4f * lmax, $"iter {i}: end length drifted, {endLen} vs {l2}");
            }

            // Force full weight to check reachability independent of the randomized blend weight.
            var fullInput = input;
            fullInput.PositionWeight = 1f;
            fullInput.RotationWeight = 1f;
            fullInput.MasterWeight = 1f;
            var fullResult = TwoBoneIkSolver.Solve(fullInput);
            TestHelpers.AssertFinite(fullResult);

            // Reachable range is [|l1-l2|, lmax*(1+maxStretch)]: closer than |l1-l2| is a genuine
            // near-side dead zone (the longer bone cannot fold back past the shorter one), just as
            // targets beyond max reach are unreachable on the far side.
            bool aboveMinReach = targetDist >= MathF.Abs(l1 - l2);
            bool withinMaxReach = targetDist <= lmax * (1f + fullInput.MaxStretch);

            if (fullInput.SoftFraction < 1e-4f && aboveMinReach && withinMaxReach)
            {
                float endErr = (fullResult.EndPosition - target).Length();
                Assert.True(endErr < 1e-3f * lmax, $"iter {i}: full-weight end error {endErr}");
            }

            Vector3 polePlaneNormal = Vector3.Normalize(Vector3.Cross(aHatApprox, Vector3.Normalize(polePerp)));
            float elbowOutOfPlane = MathF.Abs(Vector3.Dot(fullResult.MidPosition - root, polePlaneNormal));
            Assert.True(elbowOutOfPlane < 1e-2f * lmax, $"iter {i}: elbow off pole plane by {elbowOutOfPlane}");
        }
    }
}
