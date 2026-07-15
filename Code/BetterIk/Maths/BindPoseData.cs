namespace BetterIk.Maths;

using Vector3 = System.Numerics.Vector3;

/// <summary>Precomputed once per chain from the bind pose. Result of <see cref="TwoBoneIkSolver.AnalyzeBindPose"/>.</summary>
public readonly struct BindPoseData
{
    public readonly float RootBoneLength;
    public readonly float MidBoneLength;

    /// <summary>Unit bend-plane normal derived from the bind pose.</summary>
    public readonly Vector3 BendNormal;

    /// <summary>Unit, root-relative direction toward the bind-pose elbow/knee offset. This is the
    /// "elbow points backward, knee points forward" answer, read directly off the rig's own bind pose.</summary>
    public readonly Vector3 DefaultPoleDirection;

    /// <summary>False if the bind-pose chain was collinear (no elbow offset to read a direction from);
    /// the returned directions are still valid unit vectors, just not anatomically meaningful.</summary>
    public readonly bool IsReliable;

    public BindPoseData(float rootBoneLength, float midBoneLength, Vector3 bendNormal, Vector3 defaultPoleDirection, bool isReliable)
    {
        RootBoneLength = rootBoneLength;
        MidBoneLength = midBoneLength;
        BendNormal = bendNormal;
        DefaultPoleDirection = defaultPoleDirection;
        IsReliable = isReliable;
    }
}
