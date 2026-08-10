#nullable enable

using System.Diagnostics.CodeAnalysis;
using Sandbox;
using BetterIk.Maths;

namespace BetterIk;

/// <summary>
/// Counter-rotates the lower body (legs/pelvis) to face MovementDirection while the upper spine and
/// head keep their authored facing, tapering across a named spine-bone chain (see SpineBones).
/// Driven by BetterIkSystem at Stage.UpdateBones (not its own OnPreRender), after RootOffset and
/// before FootPlacementIK.
/// </summary>
public sealed class OrientationWarp : Component, IHasSkinnedRenderer
{
	[Property] public SkinnedModelRenderer? Renderer { get; set; }

	[Property, Range( 0f, 1f )] public float Weight { get; set; } = 1f;
	[Property] public float MaxWarpDegrees { get; set; } = 90f;
	[Property] public float MinSpeed { get; set; } = 10f;
	[Property] public float SmoothingRate { get; set; } = 10f;
	// Space-separated bone names, pelvis-ward first - see FootPlacementIK.IgnoreTags for the same
	// plain-string-list convention.
	[Property] public string SpineBones { get; set; } = "";

	/// <summary>Read the final post-override pose (TryGetBoneTransform) instead of the raw
	/// animation pose. Enable ONLY when a full-pose driver (e.g. motion matching) clears and
	/// rewrites every bone every frame - on such characters the animation pose is stale bind-pose
	/// data. Leave OFF for animgraph-driven characters: with nothing rewriting bones each frame,
	/// this component would read back its own previous write and compound toward infinity.</summary>
	[Property, Group( "Advanced" )] public bool UseFinalPose { get; set; } = false;

	/// <summary>World-space movement direction driving the warp target angle; set every frame by
	/// external glue code. Zero (or below MinSpeed) decays the target angle to 0.</summary>
	public Vector3 MovementDirection { get; set; }

	private (SkinnedModelRenderer Renderer, string SpineBones) _cachedSignature;
	// Only valid once EnsureResolved() has returned true at least once.
	private IReadOnlyList<BoneCollection.Bone> _allBones = null!;
	private Dictionary<int, float> _boneFactors = null!;

	private float _smoothedAngle;

	protected override void OnStart()
	{
		Renderer ??= GameObject.GetComponent<SkinnedModelRenderer>()
			?? GameObject.GetComponentInParent<SkinnedModelRenderer>( includeSelf: true );
	}

	// Overrides persist on the renderer until explicitly cleared, so disabling mid-play would
	// otherwise freeze the skeleton at its last warped pose forever on any character where nothing
	// else rewrites bones every frame.
	protected override void OnDisabled()
	{
		_smoothedAngle = 0f;
		Renderer?.ClearPhysicsBones();
	}

	/// <summary>Called once per frame by BetterIkSystem, after RootOffset and before FootPlacementIK.</summary>
	public void Apply()
	{
		if ( !EnsureResolved() )
			return;

		Vector3 up = Vector3.Up;
		float speed = MovementDirection.Length;

		float targetAngle = speed >= MinSpeed
			? OrientationWarpSolver.ComputeClampedAngle(
				Renderer.WorldRotation.Forward.ToNumerics(), MovementDirection.ToNumerics(), up.ToNumerics(), MaxWarpDegrees * (MathF.PI / 180f) )
			: 0f;

		float dt = Time.Delta;
		_smoothedAngle = FootPlacementSolver.SmoothOffset( _smoothedAngle, targetAngle, SmoothingRate, dt );

		// Snap to exact zero once within epsilon of a zero target - exponential decay never reaches
		// exact 0 on its own, which would otherwise keep the hazardous "write an unchanged value
		// back every frame" pattern alive permanently once settled (same as FootPlacementIK).
		if ( targetAngle == 0f && MathF.Abs( _smoothedAngle ) < 1e-3f )
			_smoothedAngle = 0f;

		if ( Weight <= 0f || _smoothedAngle == 0f )
			return;

		Vector3 pivot = Renderer.WorldPosition;
		float weight = Math.Clamp( Weight, 0f, 1f );

		foreach ( var bone in _allBones )
		{
			if ( !_boneFactors.TryGetValue( bone.Index, out float factor ) || factor == 0f )
				continue;

			if ( !Renderer.TryGetBonePose( in bone, UseFinalPose, out var worldTx ) )
				continue;

			var (newPos, newRot) = OrientationWarpSolver.RotateAroundPivot(
				worldTx.Position.ToNumerics(), worldTx.Rotation.ToNumerics(), pivot.ToNumerics(), up.ToNumerics(), _smoothedAngle * factor );

			var blended = new global::Transform(
				Vector3.Lerp( worldTx.Position, newPos.ToSandbox(), weight ),
				Rotation.Slerp( worldTx.Rotation, newRot.ToSandbox(), weight ) ).WithScale( worldTx.Scale );

			// SetBoneTransform expects model-local space, unlike TryGetBoneTransformAnimation/
			// TryGetBoneTransform which are documented as worldspace - see MathBridge.ToModelLocal.
			Renderer.SetBoneTransform( in bone, Renderer.ToModelLocal( blended ) );
		}

#pragma warning disable CS0612
		Renderer.PostAnimationUpdate();
#pragma warning restore CS0612
	}

	[MemberNotNullWhen( true, nameof( Renderer ) )]
	private bool EnsureResolved()
	{
		if ( Renderer is null || Renderer.Model is null )
			return false;

		var signature = (Renderer, SpineBones);
		if ( signature.Equals( _cachedSignature ) && _boneFactors is not null )
			return true;

		_allBones = Renderer.Model.Bones.AllBones;

		int count = _allBones.Count;
		var boneNames = new string[count];
		var parentIndices = new int[count];
		for ( int i = 0; i < count; i++ )
		{
			var b = _allBones[i];
			boneNames[b.Index] = b.Name;
			parentIndices[b.Index] = b.Parent?.Index ?? -1;
		}

		var spineBoneNames = SpineBones.Split( ' ', StringSplitOptions.RemoveEmptyEntries );
		_boneFactors = OrientationWarpSolver.ComputeBoneFactors( boneNames, parentIndices, spineBoneNames );
		_cachedSignature = signature;

		return true;
	}
}
