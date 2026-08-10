#nullable enable

using Sandbox;

namespace BetterIk;

/// <summary>Routes a bone pose read through either the pre-override animgraph pose or the final
/// post-override pose, per component. See UseFinalPose on each IK component for when to use which.</summary>
internal static class PoseReader
{
	public static bool TryGetBonePose( this SkinnedModelRenderer renderer,
		in BoneCollection.Bone bone, bool useFinalPose, out global::Transform tx )
		=> useFinalPose
			? renderer.TryGetBoneTransform( in bone, out tx )
			: renderer.TryGetBoneTransformAnimation( in bone, out tx );
}
