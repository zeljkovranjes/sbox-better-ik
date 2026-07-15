namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>
/// Engine-agnostic, unconstrained FABRIK solver for variable-length chains (tails, tentacles,
/// ropes). No pole vectors, no joint constraints/limits, no per-joint stiffness in this version -
/// use TwoBoneIK for hinge-like limb behavior; this is for chains that don't need it. Pure and
/// stateless: identical input always produces identical output, safe to call every frame.
/// </summary>
public static class FabrikSolver
{
    private const float TinyLenSq = 1e-12f;

    public static FabrikResult Solve(in FabrikInput input)
    {
        Vector3[] animated = input.JointPositions;
        int n = animated.Length;

        float weight = Math.Clamp(input.Weight, 0f, 1f);

        // Structural early-out: exact passthrough, no solve runs at all.
        if (weight <= 0f)
        {
            return new FabrikResult
            {
                JointPositions = CopyArray(animated),
                IterationsUsed = 0,
                Converged = false,
            };
        }

        float[] segLengths = new float[n - 1];
        float totalLength = 0f;
        for (int i = 0; i < n - 1; i++)
        {
            segLengths[i] = (animated[i + 1] - animated[i]).Length();
            totalLength += segLengths[i];
        }

        Vector3 root = animated[0];
        Vector3 toTarget = input.TargetPosition - root;
        float rootToTargetDist = toTarget.Length();

        Vector3[] solved;
        int iterationsUsed;
        bool converged;

        // Target coincides with root: aim direction is undefined, passthrough unchanged rather
        // than dividing by a zero-length vector.
        if (rootToTargetDist < 1e-5f)
        {
            solved = CopyArray(animated);
            iterationsUsed = 0;
            converged = false;
        }
        else if (rootToTargetDist >= totalLength)
        {
            // Unreachable: analytic straight chain along the root-to-target ray, deterministic.
            Vector3 dir = toTarget / rootToTargetDist;
            solved = new Vector3[n];
            solved[0] = root;
            float cumulative = 0f;
            for (int i = 1; i < n; i++)
            {
                cumulative += segLengths[i - 1];
                solved[i] = root + dir * cumulative;
            }
            iterationsUsed = 0;
            converged = false;
        }
        else
        {
            solved = CopyArray(animated);
            float tolerance = MathF.Max(input.Tolerance, 1e-6f);
            int maxIterations = Math.Max(input.MaxIterations, 1);

            converged = false;
            iterationsUsed = 0;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                iterationsUsed = iter + 1;

                // Backward pass: pin the end to the target, walk toward the root re-fixing lengths.
                solved[n - 1] = input.TargetPosition;
                for (int i = n - 2; i >= 0; i--)
                {
                    Vector3 dir = SafeDirection(solved[i] - solved[i + 1]);
                    solved[i] = solved[i + 1] + dir * segLengths[i];
                }

                // Forward pass: re-pin the root, walk toward the end re-fixing lengths.
                solved[0] = root;
                for (int i = 1; i < n; i++)
                {
                    Vector3 dir = SafeDirection(solved[i] - solved[i - 1]);
                    solved[i] = solved[i - 1] + dir * segLengths[i - 1];
                }

                if ((solved[n - 1] - input.TargetPosition).Length() <= tolerance)
                {
                    converged = true;
                    break;
                }
            }
        }

        if (weight >= 1f)
        {
            return new FabrikResult { JointPositions = solved, IterationsUsed = iterationsUsed, Converged = converged };
        }

        // Position-lerp blend. This does not preserve segment lengths at intermediate weights -
        // accepted tradeoff, same class as TwoBoneIK's weight blend; tails are visually forgiving.
        var blended = new Vector3[n];
        for (int i = 0; i < n; i++)
            blended[i] = Vector3.Lerp(animated[i], solved[i], weight);

        return new FabrikResult { JointPositions = blended, IterationsUsed = iterationsUsed, Converged = converged };
    }

    /// <summary>
    /// Derives per-joint world rotations from the change in segment direction between the animated
    /// and solved poses, via the same shortest-arc delta-rotation philosophy already proven under
    /// bone roll (IkMath.FromToRotation). No twist/roll control in this version - long chains under
    /// large deflection can accumulate visually odd roll; acceptable for v1, a twist-distribution
    /// pass is a possible later addition. The leaf bone (last index) has no segment of its own, so
    /// it is a documented convention, not a derived truth: it reuses the last real segment's delta.
    /// </summary>
    public static Quaternion[] DeriveRotations(Vector3[] animatedPositions, Vector3[] solvedPositions, Quaternion[] animatedRotations)
    {
        int n = animatedPositions.Length;
        var result = new Quaternion[n];
        Quaternion lastDelta = Quaternion.Identity;

        for (int i = 0; i < n - 1; i++)
        {
            Vector3 animatedDir = SafeDirection(animatedPositions[i + 1] - animatedPositions[i]);
            Vector3 solvedDir = SafeDirection(solvedPositions[i + 1] - solvedPositions[i]);
            lastDelta = IkMath.FromToRotation(animatedDir, solvedDir);
            result[i] = Quaternion.Normalize(lastDelta * animatedRotations[i]);
        }

        // Leaf bone convention: reuse the last real segment's delta, applied to the leaf's own
        // animated rotation (not the previous joint's).
        result[n - 1] = Quaternion.Normalize(lastDelta * animatedRotations[n - 1]);
        return result;
    }

    private static Vector3 SafeDirection(Vector3 v)
    {
        float lenSq = v.LengthSquared();
        return lenSq < TinyLenSq ? Vector3.UnitX : v / MathF.Sqrt(lenSq);
    }

    // Array.Clone() is not on s&box's code whitelist for the game-code assembly (confirmed via
    // a live compile attempt: "System.Array.Clone() is not allowed when whitelist is enabled").
    // A manual element-by-element copy is the whitelist-safe equivalent.
    private static Vector3[] CopyArray(Vector3[] source)
    {
        var copy = new Vector3[source.Length];
        for (int i = 0; i < source.Length; i++)
            copy[i] = source[i];
        return copy;
    }
}
