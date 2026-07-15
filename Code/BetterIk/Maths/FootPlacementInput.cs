namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;

/// <summary>Per-frame input to <see cref="FootPlacementSolver.Solve"/>. Angles are radians.</summary>
public struct FootPlacementInput
{
    public FootInput LeftFoot;
    public FootInput RightFoot;

    /// <summary>Character ground-plane reference (typically the model root world position).</summary>
    public Vector3 OriginPosition;

    /// <summary>World up. Normalized by the solver.</summary>
    public Vector3 UpAxis;

    /// <summary>Additive raise applied to every grounded foot's delta.</summary>
    public float FootHeightOffset;

    /// <summary>Max positive (upward) vertical correction per foot.</summary>
    public float MaxStepUp;

    /// <summary>Max negative (downward) vertical correction per foot, as a positive number.</summary>
    public float MaxStepDown;

    /// <summary>Max pelvis drop, as a positive number.</summary>
    public float MaxPelvisDrop;

    /// <summary>Max pelvis raise, usually 0.</summary>
    public float MaxPelvisRaise;

    /// <summary>Clamp on how far a foot's rotation may tilt to align with the ground normal.</summary>
    public float MaxFootRotationRadians;

    /// <summary>Ground hits steeper than this (measured from up) are treated as no-hit.</summary>
    public float MaxGroundSlopeRadians;

    /// <summary>0-1. Applied ONLY to <see cref="FootPlacementResult.PelvisOffset"/> - per-foot
    /// results are unweighted, so the caller's own two-bone solve (with its own tested,
    /// endpoint-exact weight blend) is the single place feet actually get blended.</summary>
    public float Weight;
}
