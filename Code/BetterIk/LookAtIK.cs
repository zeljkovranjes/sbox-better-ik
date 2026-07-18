#nullable enable

using System.Diagnostics.CodeAnalysis;
using Sandbox;
using BetterIk.Maths;

namespace BetterIk;

/// <summary>
/// Single-bone rotation-only look-at constraint (head/eye aim), clamped to a maximum angle from the
/// bone's currently animated aim direction. Drop on the model root (or any descendant of it), set
/// BoneName to the head/eye bone name, drag in a Target. No chain resolution needed (single bone).
/// </summary>
public sealed class LookAtIK : Component, IHasSkinnedRenderer
{
	[Property] public SkinnedModelRenderer? Renderer { get; set; }
	[Property, BoneName] public string BoneName { get; set; } = "";
	[Property] public GameObject? Target { get; set; }

	[Property, Range( 0f, 1f )] public float Weight { get; set; } = 1f;
	[Property, Range( 0f, 180f )] public float MaxAngleDegrees { get; set; } = 45f;

	// Zero = auto-derive from bind pose (bone-to-first-child offset). Non-zero overrides that.
	[Property, Group( "Advanced" )] public Vector3 LocalAimAxisOverride { get; set; } = Vector3.Zero;

	public bool HasValidBone { get; private set; }

	private (SkinnedModelRenderer Renderer, string BoneName) _cachedSignature;
	// Only valid once EnsureResolved() has returned true at least once, same as a RequireComponent
	// field is only valid once OnStart has run.
	private BoneCollection.Bone _bone = null!;
	private LookAtBindData _bindData;

	protected override void OnStart()
	{
		Renderer ??= GameObject.GetComponent<SkinnedModelRenderer>()
			?? GameObject.GetComponentInParent<SkinnedModelRenderer>( includeSelf: true );
	}

	protected override void OnPreRender()
	{
		Solve();
	}

	private void Solve()
	{
		if ( !EnsureResolved() )
		{
			HasValidBone = false;
			return;
		}

		HasValidBone = true;

		if ( Target is null || !Target.IsValid )
			return;

		// Weight <= 0 skips the read-solve-write cycle entirely - see TwoBoneIK.Solve for why
		// (confirmed live on FabrikIK: writing an "unchanged" result back every frame is not
		// perfectly lossless through the world<->model-local round trip and can compound over a
		// long session even on an animationless rig).
		if ( Weight <= 0f )
			return;

		Renderer.TryGetBoneTransformAnimation( in _bone, out var boneTx );

		Vector3 localAim = LocalAimAxisOverride != Vector3.Zero
			? LocalAimAxisOverride.Normal
			: _bindData.LocalAimDirection.ToSandbox();

		var input = new LookAtInput
		{
			BonePosition = boneTx.Position.ToNumerics(),
			BoneRotation = boneTx.Rotation.ToNumerics(),
			LocalAimDirection = localAim.ToNumerics(),
			TargetPosition = Target.WorldPosition.ToNumerics(),
			MaxAngleRadians = MaxAngleDegrees * (MathF.PI / 180f),
			Weight = Weight,
		};

		var result = LookAtSolver.Solve( input );

		// SetBoneTransform expects model-local space, unlike TryGetBoneTransformAnimation/
		// TryGetBoneTransform which are documented as worldspace - see MathBridge.ToModelLocal.
		Renderer.SetBoneTransform( in _bone, Renderer.ToModelLocal( new global::Transform( boneTx.Position, result.BoneRotation.ToSandbox() ).WithScale( boneTx.Scale ) ) );

		// PostAnimationUpdate is [Obsolete] with no replacement documented in the shipped XML docs;
		// kept defensively, matching TwoBoneIK's already-verified usage.
#pragma warning disable CS0612
		Renderer.PostAnimationUpdate();
#pragma warning restore CS0612
	}

	[MemberNotNullWhen( true, nameof( Renderer ) )]
	private bool EnsureResolved()
	{
		if ( Renderer is null || Renderer.Model is null || string.IsNullOrEmpty( BoneName ) )
			return false;

		var signature = (Renderer, BoneName);
		if ( signature.Equals( _cachedSignature ) && _bone is not null )
			return true;

		var bones = Renderer.Model.Bones;
		if ( !bones.HasBone( BoneName ) )
			return false;

		_bone = bones.GetBone( BoneName );
		_cachedSignature = signature;

		// Child's LocalTransform is already expressed relative to this bone (its parent), so it is
		// directly usable as the bone-local aim offset - no world-space composition needed.
		Vector3? childLocalPos = _bone.HasChildren
			? _bone.Children[0].LocalTransform.Position.ToNumerics()
			: null;

		_bindData = LookAtSolver.AnalyzeBindPose( System.Numerics.Vector3.Zero, System.Numerics.Quaternion.Identity, childLocalPos );

		return true;
	}

	protected override void DrawGizmos()
	{
		if ( !EnsureResolved() )
		{
			Gizmo.Draw.Color = Color.Red;
			Gizmo.Draw.WorldText( "LookAtIK: invalid bone", new global::Transform( WorldPosition ) );
			return;
		}

		Renderer.TryGetBoneTransformAnimation( in _bone, out var boneTx );

		Vector3 localAim = LocalAimAxisOverride != Vector3.Zero
			? LocalAimAxisOverride.Normal
			: _bindData.LocalAimDirection.ToSandbox();
		Vector3 aimDir = boneTx.Rotation * localAim;

		const float scale = 4f;

		Gizmo.Draw.Color = _bindData.IsReliable ? Color.Cyan : Color.Orange;
		Gizmo.Draw.Arrow( boneTx.Position, boneTx.Position + aimDir * (scale * 4f), scale * 0.5f, scale * 0.3f );

		Gizmo.Draw.Color = Color.White.WithAlpha( 0.4f );
		Vector3 coneApex = boneTx.Position + aimDir * (scale * 4f);
		float coneRadius = MathF.Tan( MaxAngleDegrees * (MathF.PI / 180f) ) * (scale * 4f);
		Gizmo.Draw.LineCircle( coneApex, aimDir, coneRadius, 0f, 360f, 32 );

		if ( Target is not null )
		{
			Gizmo.Draw.Color = Color.White;
			Gizmo.Draw.LineSphere( new Sphere( Target.WorldPosition, scale * 0.5f ) );
			Gizmo.Draw.Line( boneTx.Position, Target.WorldPosition );
		}

		if ( !_bindData.IsReliable )
		{
			Gizmo.Draw.Color = Color.Orange;
			Gizmo.Draw.WorldText( "LookAtIK: no bind-pose child, using fallback axis (see LocalAimAxisOverride)", new global::Transform( boneTx.Position + Vector3.Up * (scale * 5f) ) );
		}
	}
}
