namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

/// <summary>
/// Engine-agnostic, closed-form single-bone look-at solver. Pure and stateless: identical input
/// always produces identical output, safe to call every frame for runtime blending.
/// </summary>
public static class LookAtSolver
{
    private const float TinyLenSq = 1e-12f;

    public static LookAtBindData AnalyzeBindPose(Vector3 boneBindPosition, Quaternion boneBindRotation, Vector3? childBindPosition)
    {
        if (childBindPosition is Vector3 child)
        {
            Vector3 offset = child - boneBindPosition;
            if (offset.LengthSquared() >= TinyLenSq)
            {
                Vector3 worldAim = Vector3.Normalize(offset);
                Vector3 localAim = Vector3.Normalize(Vector3.Transform(worldAim, Quaternion.Inverse(boneBindRotation)));
                return new LookAtBindData(localAim, true);
            }
        }

        return new LookAtBindData(Vector3.UnitX, false);
    }

    public static LookAtResult Solve(in LookAtInput input)
    {
        Vector3 toTarget = input.TargetPosition - input.BonePosition;
        if (toTarget.LengthSquared() < TinyLenSq)
        {
            return new LookAtResult
            {
                BoneRotation = input.BoneRotation,
                Solved = false,
            };
        }

        Vector3 animatedAimDir = Vector3.Normalize(Vector3.Transform(input.LocalAimDirection, input.BoneRotation));
        Vector3 desiredAimDir = Vector3.Normalize(toTarget);

        float maxAngle = MathF.Max(input.MaxAngleRadians, 0f);
        float cosAngle = Math.Clamp(Vector3.Dot(animatedAimDir, desiredAimDir), -1f, 1f);
        float rawAngle = MathF.Acos(cosAngle);

        Vector3 clampedAimDir;
        if (rawAngle <= maxAngle)
        {
            clampedAimDir = desiredAimDir;
        }
        else
        {
            Vector3 axis = Vector3.Cross(animatedAimDir, desiredAimDir);
            axis = axis.LengthSquared() >= TinyLenSq
                ? Vector3.Normalize(axis)
                : IkMath.AnyPerpendicular(animatedAimDir);

            clampedAimDir = Vector3.Normalize(Vector3.Transform(animatedAimDir, Quaternion.CreateFromAxisAngle(axis, maxAngle)));
        }

        Quaternion deltaFull = IkMath.FromToRotation(animatedAimDir, clampedAimDir);
        Quaternion deltaBlended = BlendRotation(Quaternion.Identity, deltaFull, Math.Clamp(input.Weight, 0f, 1f));

        return new LookAtResult
        {
            BoneRotation = Quaternion.Normalize(deltaBlended * input.BoneRotation),
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
}
