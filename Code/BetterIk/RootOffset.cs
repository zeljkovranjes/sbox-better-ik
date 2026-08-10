#nullable enable

using Sandbox;

namespace BetterIk;

/// <summary>
/// Manual, game-code-settable whole-skeleton position+rotation correction (e.g. root-motion drift
/// correction), applied uniformly to every bone about the renderer's own origin. Defaults to
/// identity (fully inert) - no automatic drift computation, that is the caller's responsibility.
/// Driven by BetterIkSystem at Stage.UpdateBones, before OrientationWarp and FootPlacementIK.
/// </summary>
public sealed class RootOffset : Component, IHasSkinnedRenderer
{
	[Property] public SkinnedModelRenderer? Renderer { get; set; }

	[Property] public Vector3 PositionOffset { get; set; } = Vector3.Zero;
	[Property] public Angles RotationOffset { get; set; } = Angles.Zero;
	[Property, Range( 0f, 1f )] public float Weight { get; set; } = 1f;

	/// <summary>Read the final post-override pose (TryGetBoneTransform) instead of the raw
	/// animation pose. Enable ONLY when a full-pose driver (e.g. motion matching) clears and
	/// rewrites every bone every frame - on such characters the animation pose is stale bind-pose
	/// data. Leave OFF for animgraph-driven characters: with nothing rewriting bones each frame,
	/// this component would read back its own previous write and compound toward infinity.</summary>
	[Property, Group( "Advanced" )] public bool UseFinalPose { get; set; } = false;

	protected override void OnStart()
	{
		Renderer ??= GameObject.GetComponent<SkinnedModelRenderer>()
			?? GameObject.GetComponentInParent<SkinnedModelRenderer>( includeSelf: true );
	}

	// Overrides persist on the renderer until explicitly cleared, so disabling mid-play would
	// otherwise freeze the skeleton at its last offset pose forever on any character where nothing
	// else rewrites bones every frame.
	protected override void OnDisabled() => Renderer?.ClearPhysicsBones();

	/// <summary>Called once per frame by BetterIkSystem, before OrientationWarp and FootPlacementIK.</summary>
	public void Apply()
	{
		if ( Renderer is null || Renderer.Model is null )
			return;

		// Identity offset (no position, no rotation) is a fully inert no-op regardless of Weight -
		// skip entirely rather than writing back an unchanged value every bone every frame, which
		// is not lossless through the world<->model-local round trip and compounds to Infinity (see
		// the other components' Weight<=0 guards for the same hazard).
		bool positionIsZero = PositionOffset.LengthSquared < 1e-6f;
		bool rotationIsZero = RotationOffset.pitch * RotationOffset.pitch + RotationOffset.yaw * RotationOffset.yaw + RotationOffset.roll * RotationOffset.roll < 1e-6f;
		if ( Weight <= 0f || (positionIsZero && rotationIsZero) )
			return;

		Rotation offsetRotation = Rotation.From( RotationOffset );
		float weight = Math.Clamp( Weight, 0f, 1f );

		foreach ( var bone in Renderer.Model.Bones.AllBones )
		{
			if ( !Renderer.TryGetBonePose( in bone, UseFinalPose, out var worldTx ) )
				continue;

			// SetBoneTransform expects model-local space, unlike TryGetBoneTransformAnimation/
			// TryGetBoneTransform which are documented as worldspace - see MathBridge.ToModelLocal.
			// Applying the offset in that local space rotates/translates the whole skeleton as a
			// rigid body about the renderer's own origin, which is what "offset the whole skeleton
			// uniformly" requires.
			var local = Renderer.ToModelLocal( worldTx );
			var offsetLocal = new global::Transform( offsetRotation * local.Position + PositionOffset, offsetRotation * local.Rotation ).WithScale( local.Scale );

			var blended = new global::Transform(
				Vector3.Lerp( local.Position, offsetLocal.Position, weight ),
				Rotation.Slerp( local.Rotation, offsetLocal.Rotation, weight ) ).WithScale( local.Scale );

			Renderer.SetBoneTransform( in bone, blended );
		}

#pragma warning disable CS0612
		Renderer.PostAnimationUpdate();
#pragma warning restore CS0612
	}
}
