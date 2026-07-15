namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>
/// Engine-agnostic, closed-form foot-grounding solver: per-foot vertical correction and
/// ground-normal rotation alignment, plus a pelvis offset derived from both feet. Pure and
/// stateless: identical input always produces identical output, safe to call every frame.
/// </summary>
public static class FootPlacementSolver
{
    private const float TinyLenSq = 1e-12f;

    public static FootPlacementResult Solve(in FootPlacementInput input)
    {
        if (input.UpAxis.LengthSquared() < TinyLenSq)
        {
            return new FootPlacementResult
            {
                LeftFoot = new FootResult(false, 0f, input.LeftFoot.FootRotation),
                RightFoot = new FootResult(false, 0f, input.RightFoot.FootRotation),
                PelvisOffset = 0f,
                Solved = false,
            };
        }

        Vector3 up = Vector3.Normalize(input.UpAxis);

        float maxStepUp = MathF.Max(input.MaxStepUp, 0f);
        float maxStepDown = MathF.Max(input.MaxStepDown, 0f);
        float maxPelvisDrop = MathF.Max(input.MaxPelvisDrop, 0f);
        float maxPelvisRaise = MathF.Max(input.MaxPelvisRaise, 0f);
        float maxFootRotation = MathF.Max(input.MaxFootRotationRadians, 0f);
        float maxSlope = MathF.Max(input.MaxGroundSlopeRadians, 0f);
        float weight = Math.Clamp(input.Weight, 0f, 1f);

        FootResult left = SolveFoot(input.LeftFoot, up, input.OriginPosition, input.FootHeightOffset, maxStepUp, maxStepDown, maxFootRotation, maxSlope);
        FootResult right = SolveFoot(input.RightFoot, up, input.OriginPosition, input.FootHeightOffset, maxStepUp, maxStepDown, maxFootRotation, maxSlope);

        float pelvisOffset = 0f;
        bool anyGrounded = left.Grounded || right.Grounded;
        if (anyGrounded)
        {
            float pRaw = left.Grounded && right.Grounded
                ? MathF.Min(left.VerticalDelta, right.VerticalDelta)
                : left.Grounded ? left.VerticalDelta : right.VerticalDelta;

            pelvisOffset = Math.Clamp(pRaw, -maxPelvisDrop, maxPelvisRaise) * weight;
        }

        return new FootPlacementResult
        {
            LeftFoot = left,
            RightFoot = right,
            PelvisOffset = pelvisOffset,
            Solved = true,
        };
    }

    private static FootResult SolveFoot(in FootInput foot, Vector3 up, Vector3 origin, float footHeightOffset, float maxStepUp, float maxStepDown, float maxFootRotation, float maxSlope)
    {
        if (!foot.HasHit)
            return new FootResult(false, 0f, foot.FootRotation);

        if (foot.HitNormal.LengthSquared() < TinyLenSq)
            return new FootResult(false, 0f, foot.FootRotation);

        Vector3 n = Vector3.Normalize(foot.HitNormal);
        float slopeAngle = MathF.Acos(Math.Clamp(Vector3.Dot(n, up), -1f, 1f));
        if (slopeAngle > maxSlope)
            return new FootResult(false, 0f, foot.FootRotation);

        float rawDelta = Vector3.Dot(foot.HitPoint - origin, up) + footHeightOffset;
        float delta = Math.Clamp(rawDelta, -maxStepDown, maxStepUp);

        if (maxFootRotation <= 0f)
            return new FootResult(true, delta, foot.FootRotation);

        Quaternion deltaRot;
        if (slopeAngle <= maxFootRotation)
        {
            deltaRot = IkMath.FromToRotation(up, n);
        }
        else
        {
            Vector3 axis = Vector3.Cross(up, n);
            axis = axis.LengthSquared() >= TinyLenSq ? Vector3.Normalize(axis) : IkMath.AnyPerpendicular(up);
            deltaRot = Quaternion.CreateFromAxisAngle(axis, maxFootRotation);
        }

        Quaternion targetRotation = Quaternion.Normalize(deltaRot * foot.FootRotation);
        return new FootResult(true, delta, targetRotation);
    }

    /// <summary>Framerate-independent exponential approach toward target. ratePerSecond &lt;= 0
    /// disables smoothing (snaps to target); deltaTime &lt;= 0 holds at current.</summary>
    public static float SmoothOffset(float current, float target, float ratePerSecond, float deltaTime)
    {
        if (ratePerSecond <= 0f)
            return target;
        if (deltaTime <= 0f)
            return current;

        return target + (current - target) * MathF.Exp(-ratePerSecond * deltaTime);
    }
}
