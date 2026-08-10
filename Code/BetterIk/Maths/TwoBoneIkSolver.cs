namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>
/// Engine-agnostic, closed-form two-bone IK solver with pole vector control. Pure and stateless:
/// identical input always produces identical output, safe to call every frame for runtime blending.
/// </summary>
public static class TwoBoneIkSolver
{
    // All thresholds below are relative to chain length (Lmax) or another geometric quantity,
    // never absolute, so the solver is scale-equivariant.
    private const float TinyRel = 1e-6f;

    public static BindPoseData AnalyzeBindPose(Vector3 rootPos, Vector3 midPos, Vector3 endPos)
    {
        float l1 = (midPos - rootPos).Length();
        float l2 = (endPos - midPos).Length();
        float lengthSum = MathF.Max(l1 + l2, 1e-8f);

        Vector3 chainDir = IkMath.SafeNormalize(endPos - rootPos, Vector3.UnitX);
        Vector3 elbowOffset = IkMath.ProjectPerpendicular(midPos - rootPos, chainDir);

        float threshold = 1e-4f * lengthSum;

        if (elbowOffset.Length() >= threshold)
        {
            Vector3 poleDir = Vector3.Normalize(elbowOffset);
            Vector3 bendNormal = IkMath.SafeNormalize(
                Vector3.Cross(endPos - rootPos, midPos - rootPos),
                IkMath.AnyPerpendicular(chainDir));
            return new BindPoseData(l1, l2, bendNormal, poleDir, true);
        }
        else
        {
            Vector3 poleDir = IkMath.AnyPerpendicular(chainDir);
            Vector3 bendNormal = Vector3.Normalize(Vector3.Cross(chainDir, poleDir));
            return new BindPoseData(l1, l2, bendNormal, poleDir, false);
        }
    }

    public static TwoBoneIkResult Solve(in TwoBoneIkInput input)
    {
        Vector3 a = input.RootPosition;
        Vector3 b = input.MidPosition;
        Vector3 c = input.EndPosition;

        float l1 = (b - a).Length();
        float l2 = (c - b).Length();
        float lmax = l1 + l2;

        float wMaster = Math.Clamp(input.MasterWeight, 0f, 1f);
        float wPos = wMaster * Math.Clamp(input.PositionWeight, 0f, 1f);
        float wRot = wMaster * Math.Clamp(input.RotationWeight, 0f, 1f);

        Quaternion endRotation = Quaternion.Normalize(BlendRotation(input.EndRotation, input.TargetRotation, wRot));

        // Degenerate chain: at least one bone effectively zero length. Rotation goal is independent
        // of chain geometry, so it still applies; position/root/mid pass through unmodified.
        if (lmax < 1e-8f || l1 < TinyRel * lmax || l2 < TinyRel * lmax)
        {
            return new TwoBoneIkResult
            {
                RootRotation = input.RootRotation,
                MidRotation = input.MidRotation,
                EndRotation = endRotation,
                MidPosition = b,
                EndPosition = c,
                AppliedStretch = 1f,
                Solved = false,
            };
        }

        Vector3 toTarget = input.TargetPosition - a;
        float d = toTarget.Length();

        // Target coincides with root: root-to-target direction is undefined, fall back to the
        // animated chain direction (never NaN; continuity through this exact point is not required).
        Vector3 aHat = d < TinyRel * lmax
            ? IkMath.SafeNormalize(c - a, Vector3.UnitX)
            : toTarget / d;

        (float dEff, float sFull, float dFinal) = SolveReach(d, lmax, input.SoftFraction, input.MaxStretch);

        float l1s = sFull * l1;
        float l2s = sFull * l2;

        // Near-side clamp: when bone lengths differ, the chain also cannot reach closer than
        // |l1s - l2s| (folding the longer bone back over the shorter one). Mirrors the far-side
        // max-reach clamp; without it cosAlpha blows outside [-1,1] and the elbow snaps to an
        // inconsistent pose whose end position drifts off target. A near-side reach clamp found
        // by fuzz testing, not covered by the original edge-case table (which only listed
        // "beyond max reach" and "exactly at zero").
        float minReach = MathF.Abs(l1s - l2s);
        if (dFinal < minReach)
            dFinal = minReach;

        Vector3 dHat = SelectElbowDirection(input, a, b, aHat, lmax);

        float alpha = SolveRootAngle(dFinal, l1s, l2s, lmax);

        Vector3 midSolved = a + l1s * (MathF.Cos(alpha) * aHat + MathF.Sin(alpha) * dHat);
        Vector3 endSolved = a + dFinal * aHat;

        Vector3 rawPoseNormal = Vector3.Cross(c - a, b - a);
        Vector3 oldNormalHint = rawPoseNormal.LengthSquared() >= (1e-5f * l1 * l2) * (1e-5f * l1 * l2)
            ? rawPoseNormal
            : input.FallbackBendNormal;

        Vector3 newNormalHint = Vector3.Cross(aHat, dHat);

        Quaternion deltaRootFull = IkMath.DeltaRotation(b - a, oldNormalHint, midSolved - a, newNormalHint);
        Quaternion deltaMidFull = IkMath.DeltaRotation(c - b, oldNormalHint, endSolved - midSolved, newNormalHint);

        Quaternion deltaRoot = BlendRotation(Quaternion.Identity, deltaRootFull, wPos);
        Quaternion deltaMid = BlendRotation(Quaternion.Identity, deltaMidFull, wPos);

        float sBlended = 1f + (sFull - 1f) * wPos;

        // At full positional weight the solved points are the authoritative result. Rebuilding
        // them through two float quaternion rotations introduces enough rounding to flatten the
        // far end of the soft-reach curve even while the requested reach is still increasing.
        Vector3 midBlended = wPos >= 1f
            ? midSolved
            : a + sBlended * Vector3.Transform(b - a, deltaRoot);
        Vector3 endBlended = wPos >= 1f
            ? endSolved
            : midBlended + sBlended * Vector3.Transform(c - b, deltaMid);

        return new TwoBoneIkResult
        {
            RootRotation = Quaternion.Normalize(deltaRoot * input.RootRotation),
            MidRotation = Quaternion.Normalize(deltaMid * input.MidRotation),
            EndRotation = endRotation,
            MidPosition = midBlended,
            EndPosition = endBlended,
            AppliedStretch = sBlended,
            Solved = true,
        };
    }

    // Exact at t=0/t=1 (no Slerp numerical noise at the endpoints); Slerp in between.
    private static Quaternion BlendRotation(Quaternion from, Quaternion to, float t)
    {
        if (t <= 0f) return from;
        if (t >= 1f) return to;
        return Quaternion.Slerp(from, to, t);
    }

    // Soft clamp (exponential falloff) + stretch. Returns the reach distance actually solved for
    // (dEff), the blended-in stretch scale (sFull), and the final root-to-end distance (dFinal).
    private static (float dEff, float sFull, float dFinal) SolveReach(float d, float lmax, float softFractionRaw, float maxStretchRaw)
    {
        float softFraction = Math.Clamp(softFractionRaw, 0f, 0.499f);
        float maxStretch = MathF.Max(maxStretchRaw, 0f);

        float dEff;
        if (softFraction < 1e-4f)
        {
            dEff = MathF.Min(d, lmax);
        }
        else
        {
            float dSoft = (1f - softFraction) * lmax;
            float r = softFraction * lmax;
            // Evaluate the subtractive exponential in double precision. In float,
            // 1 - exp(-x) loses the remaining tail early and creates a visible reach plateau.
            dEff = d <= dSoft
                ? d
                : (float)(dSoft + r * (1d - Math.Exp(-(double)(d - dSoft) / r)));
        }

        float sFull = dEff > 0f ? Math.Clamp(d / dEff, 1f, 1f + maxStretch) : 1f;
        float dFinal = MathF.Min(d, dEff * sFull);

        return (dEff, sFull, dFinal);
    }

    // Law of cosines for the angle at the root, guarded against the near-zero-reach fold singularity
    // (where the denominator would be zero); alpha = pi/2 there safely folds the chain via dHat.
    private static float SolveRootAngle(float dFinal, float l1s, float l2s, float lmax)
    {
        if (dFinal < TinyRel * lmax || l1s < TinyRel * lmax)
            return MathF.PI / 2f;

        float cosAlpha = (dFinal * dFinal + l1s * l1s - l2s * l2s) / (2f * dFinal * l1s);
        return MathF.Acos(Math.Clamp(cosAlpha, -1f, 1f));
    }

    // Pole hint -> pose-derived elbow offset -> fallback bend normal -> arbitrary perpendicular.
    private static Vector3 SelectElbowDirection(in TwoBoneIkInput input, Vector3 a, Vector3 b, Vector3 aHat, float lmax)
    {
        if (input.HasPole)
        {
            Vector3 pPerp = IkMath.ProjectPerpendicular(input.PoleHint, aHat);
            float threshold = 1e-4f * MathF.Max(input.PoleHint.Length(), lmax);
            if (pPerp.Length() >= threshold)
                return ApplyOffset(Vector3.Normalize(pPerp), aHat, input.PoleAngleOffsetRadians);
        }

        Vector3 mPerp = IkMath.ProjectPerpendicular(b - a, aHat);
        if (mPerp.Length() >= 1e-5f * lmax)
            return ApplyOffset(Vector3.Normalize(mPerp), aHat, input.PoleAngleOffsetRadians);

        // FallbackBendNormal is a plane normal, not an in-plane direction: cross with aHat to get
        // the in-plane, perpendicular-to-aHat elbow direction it implies.
        Vector3 crossFallback = Vector3.Cross(input.FallbackBendNormal, aHat);
        Vector3 dHat = crossFallback.LengthSquared() >= 1e-10f
            ? Vector3.Normalize(crossFallback)
            : IkMath.AnyPerpendicular(aHat);

        return ApplyOffset(dHat, aHat, input.PoleAngleOffsetRadians);
    }

    private static Vector3 ApplyOffset(Vector3 dHat, Vector3 axis, float angleRadians)
    {
        if (angleRadians == 0f)
            return dHat;
        return Vector3.Transform(dHat, Quaternion.CreateFromAxisAngle(axis, angleRadians));
    }
}
