namespace BetterIk.Maths;

/// <summary>
/// Engine-agnostic plant-to-ground correction, the closed-form core of FootPlacementIK's
/// plant mode. It is deliberately separate from <see cref="FootPlacementSolver"/>: this mode
/// does not raycast arbitrary geometry, does not align a foot to a surface normal, and does
/// not move the pelvis. It answers one question per foot: given a plant point (e.g. the ball
/// of the foot) that HOVERS some measured height above the ground it was authored to touch,
/// how far down must the foot move so its planted moments meet the ground while its authored
/// lifts ride on top?
///
/// The caller supplies the TRAILING-MINIMUM plant-point height over a recent window (see
/// <see cref="PlantWindow"/>): the lowest the plant point reached recently is its planted
/// level, and only that constant is removed. Authored step lifts and heel raises above the
/// planted level survive untouched, flat-authored feet come out flat, and nothing is ever
/// rotated. Pure and stateless: identical input always produces identical output.
/// </summary>
public static class PlantToGroundSolver
{
    /// <summary>
    /// Vertical correction (world units, POSITIVE means lower the foot by this much) that
    /// lands the planted level at <paramref name="plantRestHeight"/> above the ground.
    /// </summary>
    /// <param name="trailingMinHeightAboveGround">Lowest plant-point height above the ground
    /// over the recent window, measured along the up axis. This is the "planted" level of the
    /// authored motion.</param>
    /// <param name="plantRestHeight">Height the plant point should hold above the ground when
    /// planted (its bind height on the target model). 0 puts the plant point exactly on the
    /// ground; a ball-of-foot plant point uses that bone's small bind height.</param>
    /// <param name="maxCorrection">Upper clamp on the correction. The lower clamp is fixed at
    /// 0, so a plant point already at or below its rest height is never RAISED: hover is only
    /// ever removed, never added (a grounded clip is never lifted off the canvas).</param>
    public static float ComputeCorrection(float trailingMinHeightAboveGround, float plantRestHeight, float maxCorrection)
    {
        if (maxCorrection < 0f)
            maxCorrection = 0f;
        float hover = trailingMinHeightAboveGround - plantRestHeight;
        return Math.Clamp(hover, 0f, maxCorrection);
    }
}
