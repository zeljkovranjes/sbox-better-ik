#nullable enable

using System.Diagnostics.CodeAnalysis;
using Sandbox;
using BetterIk.Maths;
using BetterIk.Skeleton;

namespace BetterIk;

/// <summary>
/// Runtime two-bone IK (arm/leg) with pole vector control. Drop on the model root (or any
/// descendant of it - the component finds the SkinnedModelRenderer via GetComponentInParent),
/// set EndBone to the hand/foot bone name, optionally drag in a Target and PoleTarget. Walks
/// up the skeleton automatically to find the mid and root bones; no per-model setup needed.
///
/// EMPIRICAL VERIFICATION CHECKLIST (see DECISIONS.md for full detail on each item):
/// 1. NON-BLOCKING, exact frame ordering not directly measured, but the component is confirmed
///    to produce correct, stable, converged position and rotation results every frame on a real
///    Citizen rig (see item 5), so OnPreRender() works well enough in practice as written.
/// 2. CONFIRMED via engine XML docs: TryGetBoneTransformAnimation returns "the worldspace bone
///    transform after animation but before physics and procedural bones" - safe for multi-limb.
/// 3. NON-BLOCKING, still technically [Obsolete] with no documented replacement; kept defensively
///    (see the pragma below). Live testing shows it causes no observable problem as currently used.
/// 4. NON-BLOCKING - not directly tested, but auto-walk resolves correctly on Citizen's real
///    hierarchy in practice (see item 5).
/// 5. CONFIRMED live on a real Citizen model: two TwoBoneIK components (right arm EndBone=hand_R,
///    left arm EndBone=hand_L) with independent targets, entered play simultaneously - both hands
///    moved correctly toward their own targets with zero compile/runtime errors. hand_R's solved
///    distance from root matched the analytic far-reach clamp (Lmax) to close numerical precision.
/// </summary>
public sealed class TwoBoneIK : Component
{
	// --- Setup (the happy path: add component, set EndBone, drag Target, done) ---
	[Property] public SkinnedModelRenderer? Renderer { get; set; }
	[Property] public string EndBone { get; set; } = "";
	[Property] public GameObject? Target { get; set; }
	[Property] public GameObject? PoleTarget { get; set; }

	// --- Weights (all lerp-safe every frame) ---
	[Property, Range( 0f, 1f )] public float Weight { get; set; } = 1f;
	[Property, Range( 0f, 1f )] public float PositionWeight { get; set; } = 1f;
	[Property, Range( 0f, 1f )] public float RotationWeight { get; set; } = 1f;

	// --- Advanced: manual bone overrides, pole fine-tuning, soft/stretch (off by default) ---
	[Property, Group( "Advanced" )] public string RootBoneOverride { get; set; } = "";
	[Property, Group( "Advanced" )] public string MidBoneOverride { get; set; } = "";
	[Property, Group( "Advanced" ), Range( -180f, 180f )] public float PoleAngleOffsetDegrees { get; set; } = 0f;
	[Property, Group( "Advanced" ), Range( 0f, 0.49f )] public float SoftFraction { get; set; } = 0f;
	[Property, Group( "Advanced" ), Range( 0f, 1f )] public float MaxStretch { get; set; } = 0f;

	// --- Diagnostics ---
	public bool HasValidChain { get; private set; }
	public bool LastSolveApplied { get; private set; }

	private (SkinnedModelRenderer Renderer, string EndBone, string RootOverride, string MidOverride) _cachedSignature;
	// Only valid once EnsureResolved() has returned true at least once, same as a RequireComponent
	// field is only valid once OnStart has run.
	private BoneCollection.Bone _rootBone = null!;
	private BoneCollection.Bone _midBone = null!;
	private BoneCollection.Bone _endBone = null!;
	private BindPoseData _bindPose;
	private Vector3 _lastPoleDirection = Vector3.Up;

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
			HasValidChain = false;
			LastSolveApplied = false;
			return;
		}

		HasValidChain = true;

		if ( Target is null || !Target.IsValid )
		{
			LastSolveApplied = false;
			return;
		}

		// Weight <= 0 skips the read-solve-write cycle entirely rather than reading the animated
		// pose and writing an "unchanged" result back. Confirmed live on FabrikIK (see
		// DECISIONS.md) that the latter is NOT perfectly lossless through the world<->model-local
		// round trip every frame - on a rig with no real animation driving it, this compounded
		// into Infinity within a few seconds even though the math is a no-op at Weight=0. Not
		// writing at all when there is nothing to blend removes the feedback path entirely.
		if ( Weight <= 0f )
		{
			LastSolveApplied = false;
			return;
		}

		Renderer.TryGetBoneTransformAnimation( in _rootBone, out var rootTx );
		Renderer.TryGetBoneTransformAnimation( in _midBone, out var midTx );
		Renderer.TryGetBoneTransformAnimation( in _endBone, out var endTx );

		Vector3 poleHint = PoleTarget is not null && PoleTarget.IsValid
			? PoleTarget.WorldPosition - rootTx.Position
			: rootTx.Rotation * _bindPose.DefaultPoleDirection.ToSandbox();

		_lastPoleDirection = poleHint.Normal;

		var input = new TwoBoneIkInput
		{
			RootPosition = rootTx.Position.ToNumerics(),
			MidPosition = midTx.Position.ToNumerics(),
			EndPosition = endTx.Position.ToNumerics(),
			RootRotation = rootTx.Rotation.ToNumerics(),
			MidRotation = midTx.Rotation.ToNumerics(),
			EndRotation = endTx.Rotation.ToNumerics(),
			TargetPosition = Target.WorldPosition.ToNumerics(),
			TargetRotation = Target.WorldRotation.ToNumerics(),
			HasPole = true, // poleHint always carries either the real pole or the bind-pose default
			PoleHint = poleHint.ToNumerics(),
			PoleAngleOffsetRadians = PoleAngleOffsetDegrees * (MathF.PI / 180f),
			FallbackBendNormal = _bindPose.BendNormal,
			PositionWeight = PositionWeight,
			RotationWeight = RotationWeight,
			MasterWeight = Weight,
			SoftFraction = SoftFraction,
			MaxStretch = MaxStretch,
		};

		var result = TwoBoneIkSolver.Solve( input );

		// SetBoneTransform expects model-local space, unlike TryGetBoneTransformAnimation/
		// TryGetBoneTransform which are documented as worldspace - see MathBridge.ToModelLocal.
		Renderer.SetBoneTransform( in _rootBone, Renderer.ToModelLocal( new global::Transform( rootTx.Position, result.RootRotation.ToSandbox() ).WithScale( rootTx.Scale ) ) );
		Renderer.SetBoneTransform( in _midBone, Renderer.ToModelLocal( new global::Transform( result.MidPosition.ToSandbox(), result.MidRotation.ToSandbox() ).WithScale( midTx.Scale ) ) );
		Renderer.SetBoneTransform( in _endBone, Renderer.ToModelLocal( new global::Transform( result.EndPosition.ToSandbox(), result.EndRotation.ToSandbox() ).WithScale( endTx.Scale ) ) );

		// PostAnimationUpdate is [Obsolete] with no replacement documented in the shipped XML
		// docs; kept defensively until checklist item 3 is verified against a live editor.
#pragma warning disable CS0612
		Renderer.PostAnimationUpdate();
#pragma warning restore CS0612

		LastSolveApplied = result.Solved;
	}

	[MemberNotNullWhen( true, nameof( Renderer ) )]
	private bool EnsureResolved()
	{
		if ( Renderer is null || Renderer.Model is null || string.IsNullOrEmpty( EndBone ) )
			return false;

		var signature = (Renderer, EndBone, RootBoneOverride, MidBoneOverride);
		if ( signature.Equals( _cachedSignature ) && _rootBone is not null )
			return true;

		var bones = Renderer.Model.Bones;
		if ( !bones.HasBone( EndBone ) )
			return false;

		IBoneNode endNode = new SandboxBoneNode( bones.GetBone( EndBone ) );

		IBoneNode? rootOverrideNode = null;
		if ( !string.IsNullOrEmpty( RootBoneOverride ) )
		{
			if ( !bones.HasBone( RootBoneOverride ) )
				return false;
			rootOverrideNode = new SandboxBoneNode( bones.GetBone( RootBoneOverride ) );
		}

		IBoneNode? midOverrideNode = null;
		if ( !string.IsNullOrEmpty( MidBoneOverride ) )
		{
			if ( !bones.HasBone( MidBoneOverride ) )
				return false;
			midOverrideNode = new SandboxBoneNode( bones.GetBone( MidBoneOverride ) );
		}

		var chain = BoneChainResolver.Resolve( endNode, rootOverrideNode, midOverrideNode );
		if ( !chain.Success )
			return false;

		_rootBone = ((SandboxBoneNode)chain.Root!).Bone;
		_midBone = ((SandboxBoneNode)chain.Mid!).Bone;
		_endBone = ((SandboxBoneNode)chain.End!).Bone;
		_cachedSignature = signature;

		// Bind-pose world positions, composed from an arbitrary origin at the root - only the
		// relative offsets matter to AnalyzeBindPose (translation-invariant), so we don't need
		// to walk all the way up to the model's true origin.
		var rootBindWorld = global::Transform.Zero;
		var midBindWorld = global::Transform.Concat( rootBindWorld, _midBone.LocalTransform );
		var endBindWorld = global::Transform.Concat( midBindWorld, _endBone.LocalTransform );

		_bindPose = TwoBoneIkSolver.AnalyzeBindPose(
			rootBindWorld.Position.ToNumerics(),
			midBindWorld.Position.ToNumerics(),
			endBindWorld.Position.ToNumerics() );

		return true;
	}

	protected override void DrawGizmos()
	{
		if ( !EnsureResolved() )
		{
			Gizmo.Draw.Color = Color.Red;
			Gizmo.Draw.WorldText( "TwoBoneIK: invalid bone chain", new global::Transform( WorldPosition ) );
			return;
		}

		if ( Target is null || !Target.IsValid )
			return;

		Renderer.TryGetBoneTransformAnimation( in _rootBone, out var rootTx );
		Renderer.TryGetBoneTransformAnimation( in _midBone, out var midTx );
		Renderer.TryGetBoneTransformAnimation( in _endBone, out var endTx );

		float l1 = (midTx.Position - rootTx.Position).Length;
		float l2 = (endTx.Position - midTx.Position).Length;
		float lmax = l1 + l2;
		float d = (Target.WorldPosition - rootTx.Position).Length;
		float minReach = MathF.Abs( l1 - l2 );
		float softStart = (1f - SoftFraction) * lmax;

		Color reachColor;
		if ( d < minReach || d > lmax * (1f + MaxStretch) )
			reachColor = Color.Red;
		else if ( SoftFraction > 0f && d > softStart )
			reachColor = Color.Yellow;
		else
			reachColor = Color.Green;

		// Solved chain (post-IK), reusing the same read this frame's Solve() already wrote.
		Renderer.TryGetBoneTransform( in _rootBone, out var solvedRoot );
		Renderer.TryGetBoneTransform( in _midBone, out var solvedMid );
		Renderer.TryGetBoneTransform( in _endBone, out var solvedEnd );

		Gizmo.Draw.Color = reachColor;
		Gizmo.Draw.Line( solvedRoot.Position, solvedMid.Position );
		Gizmo.Draw.Line( solvedMid.Position, solvedEnd.Position );

		Gizmo.Draw.Color = Color.White;
		Gizmo.Draw.LineSphere( new Sphere( Target.WorldPosition, MathF.Max( lmax * 0.05f, 1f ) ) );

		Gizmo.Draw.Color = PoleTarget is not null && PoleTarget.IsValid ? Color.White : Color.Cyan;
		Gizmo.Draw.Arrow( solvedMid.Position, solvedMid.Position + _lastPoleDirection * (lmax * 0.3f), lmax * 0.05f, lmax * 0.03f );

		Gizmo.Draw.Color = Color.Cyan.WithAlpha( 0.15f );
		Gizmo.Draw.SolidTriangle( new Triangle { A = solvedRoot.Position, B = solvedMid.Position, C = solvedEnd.Position } );

		Gizmo.Draw.Color = Color.White;
		Gizmo.Draw.WorldText( $"W:{Weight:F2} Pos:{PositionWeight:F2} Rot:{RotationWeight:F2}", new global::Transform( solvedMid.Position + Vector3.Up * (lmax * 0.1f) ) );
	}
}
