#nullable enable

using System;
using System.Diagnostics.CodeAnalysis;
using Sandbox;
using BetterIk.Maths;
using BetterIk.Skeleton;

namespace BetterIk;

/// <summary>
/// Foot grounding: traces down under each foot, plants feet on real geometry, drops the pelvis
/// when the ground is lower so the far foot can reach, and aligns foot rotation to the surface
/// normal. Drop on the model root (or any descendant), set LeftFootBone/RightFootBone to the
/// ankle/foot bone names. Root and mid bones for each leg are auto-walked like TwoBoneIK; the
/// pelvis is auto-derived as the nearest common ancestor of the two resolved leg roots.
/// </summary>
public sealed class FootPlacementIK : Component, IHasSkinnedRenderer
{
	[Property] public SkinnedModelRenderer? Renderer { get; set; }
	[Property, BoneName] public string LeftFootBone { get; set; } = "";
	[Property, BoneName] public string RightFootBone { get; set; } = "";

	[Property, Range( 0f, 1f )] public float Weight { get; set; } = 1f;
	[Property, Range( 0f, 1f )] public float FootRotationWeight { get; set; } = 1f;

	[Property, Group( "Ground" )] public float MaxStepUp { get; set; } = 18f;
	[Property, Group( "Ground" )] public float MaxStepDown { get; set; } = 18f;
	[Property, Group( "Ground" )] public float MaxPelvisDrop { get; set; } = 16f;
	[Property, Group( "Ground" )] public float MaxPelvisRaise { get; set; } = 0f;
	[Property, Group( "Ground" ), Range( 0f, 90f )] public float MaxFootRotationDegrees { get; set; } = 30f;
	[Property, Group( "Ground" ), Range( 0f, 90f )] public float MaxGroundSlopeDegrees { get; set; } = 60f;
	[Property, Group( "Ground" )] public float FootHeightOffset { get; set; } = 0f;
	[Property, Group( "Ground" )] public float SmoothingRate { get; set; } = 10f;
	[Property, Group( "Ground" )] public string IgnoreTags { get; set; } = "player";

	// ---- Plant-to-ground mode (opt-in) -----------------------------------------------------
	// The default (terrain) mode preserves the animation's authored height above an assumed
	// flat ground and only adapts to terrain deviation, which is correct for grounded
	// animations. Plant mode instead corrects animations whose feet HOVER above the ground
	// they were authored to touch (a per-clip constant conversion offset, possibly different
	// per foot): each foot's plant point (an optional plant bone, e.g. the ball of the foot;
	// the foot bone itself when unset) is measured against the ground, its TRAILING-MINIMUM
	// height over PlantWindowSeconds is treated as the hover to remove, and the foot is
	// lowered vertically by that amount with its AUTHORED ROTATION UNTOUCHED. Removing only
	// the trailing minimum keeps every within-window articulation intact: authored step
	// lifts and heel raises ride on top of the corrected plant level, flat-authored feet
	// come out flat, and nothing is ever rotated onto the surface. Pelvis is not moved in
	// plant mode (correct a shared full-body offset at the model root instead).
	[Property, Group( "Plant" )] public bool PlantToGround { get; set; } = false;
	[Property, BoneName, Group( "Plant" )] public string LeftPlantBone { get; set; } = "";
	[Property, BoneName, Group( "Plant" )] public string RightPlantBone { get; set; } = "";
	/// <summary>Rest height of the plant point above the ground when planted (its bind height).</summary>
	[Property, Group( "Plant" )] public float PlantHeightOffset { get; set; } = 0f;
	/// <summary>Trailing window (seconds) whose minimum plant-point height is removed as hover.</summary>
	[Property, Group( "Plant" )] public float PlantWindowSeconds { get; set; } = 0.8f;
	/// <summary>Upper clamp on the per-foot plant correction.</summary>
	[Property, Group( "Plant" )] public float PlantMaxCorrection { get; set; } = 6f;
	/// <summary>Skip tracing and use a flat world-space ground plane at FlatGroundHeight.</summary>
	[Property, Group( "Plant" )] public bool UseFlatGround { get; set; } = false;
	[Property, Group( "Plant" )] public float FlatGroundHeight { get; set; } = 0f;

	/// <summary>Live plant corrections (world units, positive = foot lowered), for diagnostics.</summary>
	public float CurrentPlantCorrectionL { get; private set; }
	public float CurrentPlantCorrectionR { get; private set; }
	/// <summary>Live plant-point heights above the ground before correction, for diagnostics.</summary>
	public float LastPlantResidualL { get; private set; }
	public float LastPlantResidualR { get; private set; }

	[Property, BoneName, Group( "Advanced" )] public string PelvisBoneOverride { get; set; } = "";
	[Property, Group( "Advanced" )] public GameObject? LeftPoleTarget { get; set; }
	[Property, Group( "Advanced" )] public GameObject? RightPoleTarget { get; set; }
	[Property, BoneName, Group( "Advanced" )] public string LeftRootOverride { get; set; } = "";
	[Property, BoneName, Group( "Advanced" )] public string LeftMidOverride { get; set; } = "";
	[Property, BoneName, Group( "Advanced" )] public string RightRootOverride { get; set; } = "";
	[Property, BoneName, Group( "Advanced" )] public string RightMidOverride { get; set; } = "";

	public bool HasValidChains { get; private set; }
	public bool PelvisResolved { get; private set; }
	public bool LeftGrounded { get; private set; }
	public bool RightGrounded { get; private set; }
	public float CurrentPelvisOffset { get; private set; }

	private (SkinnedModelRenderer Renderer, string LeftFootBone, string RightFootBone, string LeftRootOverride, string LeftMidOverride, string RightRootOverride, string RightMidOverride, string PelvisBoneOverride, string LeftPlantBone, string RightPlantBone) _cachedSignature;

	// Only valid once EnsureResolved() has returned true at least once.
	private BoneCollection.Bone _leftRoot = null!;
	private BoneCollection.Bone _leftMid = null!;
	private BoneCollection.Bone _leftEnd = null!;
	private BoneCollection.Bone _rightRoot = null!;
	private BoneCollection.Bone _rightMid = null!;
	private BoneCollection.Bone _rightEnd = null!;
	private BoneCollection.Bone _pelvisBone = null!;

	// Plant points (plant mode): the ball / toe bone whose ground contact is planted. Falls
	// back to the leg's own end bone when the name is unset or not present on the model.
	private BoneCollection.Bone _leftPlant = null!;
	private BoneCollection.Bone _rightPlant = null!;
	private bool _leftPlantResolved;
	private bool _rightPlantResolved;

	private BindPoseData _leftBindPose;
	private BindPoseData _rightBindPose;

	private float _smoothPelvis;
	private float _smoothDeltaL;
	private float _smoothDeltaR;

	// Plant mode state: trailing-minimum windows over each plant point's height above ground,
	// and the smoothed per-foot downward correction actually applied.
	private PlantWindow _plantWindowL;
	private PlantWindow _plantWindowR;
	private float _smoothPlantL;
	private float _smoothPlantR;

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
			HasValidChains = false;
			LeftGrounded = false;
			RightGrounded = false;
			return;
		}

		HasValidChains = true;

		// Weight <= 0 skips the entire read-trace-solve-write cycle: a SetBoneTransform whose
		// value has no restoring force (not animated, not converging toward a fixed target) is
		// not bit-lossless at the engine write boundary and compounds to Infinity over enough
		// frames (confirmed via FabrikIK). Zero the smoothed state so re-enabling Weight later
		// doesn't replay stale offsets.
		if ( Weight <= 0f )
		{
			_smoothPelvis = 0f;
			_smoothDeltaL = 0f;
			_smoothDeltaR = 0f;
			_smoothPlantL = 0f;
			_smoothPlantR = 0f;
			_plantWindowL?.Reset();
			_plantWindowR?.Reset();
			LeftGrounded = false;
			RightGrounded = false;
			CurrentPelvisOffset = 0f;
			CurrentPlantCorrectionL = 0f;
			CurrentPlantCorrectionR = 0f;
			return;
		}

		if ( PlantToGround )
		{
			SolvePlant();
			return;
		}

		Renderer.TryGetBoneTransformAnimation( in _leftRoot, out var leftRootTx );
		Renderer.TryGetBoneTransformAnimation( in _leftMid, out var leftMidTx );
		Renderer.TryGetBoneTransformAnimation( in _leftEnd, out var leftEndTx );
		Renderer.TryGetBoneTransformAnimation( in _rightRoot, out var rightRootTx );
		Renderer.TryGetBoneTransformAnimation( in _rightMid, out var rightMidTx );
		Renderer.TryGetBoneTransformAnimation( in _rightEnd, out var rightEndTx );

		var pelvisTx = global::Transform.Zero;
		if ( PelvisResolved )
			Renderer.TryGetBoneTransformAnimation( in _pelvisBone, out pelvisTx );

		Vector3 up = Vector3.Up;
		Vector3 origin = Renderer.WorldPosition;

		var leftTrace = TraceFoot( leftEndTx.Position, up, origin );
		var rightTrace = TraceFoot( rightEndTx.Position, up, origin );

		var input = new FootPlacementInput
		{
			LeftFoot = new FootInput
			{
				FootPosition = leftEndTx.Position.ToNumerics(),
				FootRotation = leftEndTx.Rotation.ToNumerics(),
				HasHit = leftTrace.Hit,
				HitPoint = leftTrace.HitPosition.ToNumerics(),
				HitNormal = leftTrace.Normal.ToNumerics(),
			},
			RightFoot = new FootInput
			{
				FootPosition = rightEndTx.Position.ToNumerics(),
				FootRotation = rightEndTx.Rotation.ToNumerics(),
				HasHit = rightTrace.Hit,
				HitPoint = rightTrace.HitPosition.ToNumerics(),
				HitNormal = rightTrace.Normal.ToNumerics(),
			},
			OriginPosition = origin.ToNumerics(),
			UpAxis = up.ToNumerics(),
			FootHeightOffset = FootHeightOffset,
			MaxStepUp = MaxStepUp,
			MaxStepDown = MaxStepDown,
			MaxPelvisDrop = MaxPelvisDrop,
			MaxPelvisRaise = MaxPelvisRaise,
			MaxFootRotationRadians = MaxFootRotationDegrees * (MathF.PI / 180f),
			MaxGroundSlopeRadians = MaxGroundSlopeDegrees * (MathF.PI / 180f),
			Weight = Weight,
		};

		var result = FootPlacementSolver.Solve( input );

		float dt = Time.Delta;
		_smoothPelvis = FootPlacementSolver.SmoothOffset( _smoothPelvis, result.PelvisOffset, SmoothingRate, dt );
		_smoothDeltaL = FootPlacementSolver.SmoothOffset( _smoothDeltaL, result.LeftFoot.VerticalDelta, SmoothingRate, dt );
		_smoothDeltaR = FootPlacementSolver.SmoothOffset( _smoothDeltaR, result.RightFoot.VerticalDelta, SmoothingRate, dt );

		// Snap to exact zero once within epsilon of a zero target: exponential decay (SmoothOffset)
		// never reaches exact 0 on its own, which would otherwise keep the hazardous "write back
		// an unchanged value forever" pattern alive permanently on flat ground / an ungrounded foot.
		if ( result.PelvisOffset == 0f && MathF.Abs( _smoothPelvis ) < 1e-3f )
			_smoothPelvis = 0f;
		if ( result.LeftFoot.VerticalDelta == 0f && MathF.Abs( _smoothDeltaL ) < 1e-3f )
			_smoothDeltaL = 0f;
		if ( result.RightFoot.VerticalDelta == 0f && MathF.Abs( _smoothDeltaR ) < 1e-3f )
			_smoothDeltaR = 0f;

		LeftGrounded = result.LeftFoot.Grounded;
		RightGrounded = result.RightFoot.Grounded;
		CurrentPelvisOffset = _smoothPelvis;

		// Skip the pelvis write entirely once its offset has snapped to exact zero - a zero-offset
		// write targets exactly the animated pose anyway, so skipping it is not a behavior change,
		// only a removal of an unforced no-op write (see the principle above).
		if ( PelvisResolved && _smoothPelvis != 0f )
		{
			// SetBoneTransform expects model-local space - see MathBridge.ToModelLocal.
			Renderer.SetBoneTransform( in _pelvisBone, Renderer.ToModelLocal( new global::Transform( pelvisTx.Position + up * _smoothPelvis, pelvisTx.Rotation ).WithScale( pelvisTx.Scale ) ) );
		}

		Vector3 pelvisShift = up * (PelvisResolved ? _smoothPelvis : 0f);

		// Skip a leg's writes only when there is genuinely nothing driving them: ungrounded (no
		// trace hit), its smoothed delta has snapped to zero, AND the pelvis isn't shifting it
		// either. A nonzero pelvis shift still requires the leg solve even for an otherwise-settled
		// ungrounded foot, since the leg still needs to follow the pelvis. Otherwise, an ungrounded
		// foot's TargetPosition is derived from this frame's own read (endTx.Position + up*0), which
		// has no restoring force once settled - the same unforced-write hazard as the pelvis case.
		bool leftNeedsWrite = result.LeftFoot.Grounded || _smoothDeltaL != 0f || pelvisShift != Vector3.Zero;
		bool rightNeedsWrite = result.RightFoot.Grounded || _smoothDeltaR != 0f || pelvisShift != Vector3.Zero;

		if ( leftNeedsWrite )
			SolveLeg( in _leftRoot, in _leftMid, in _leftEnd, leftRootTx, leftMidTx, leftEndTx, _leftBindPose, pelvisShift, _smoothDeltaL, result.LeftFoot.TargetRotation, LeftPoleTarget, up );
		if ( rightNeedsWrite )
			SolveLeg( in _rightRoot, in _rightMid, in _rightEnd, rightRootTx, rightMidTx, rightEndTx, _rightBindPose, pelvisShift, _smoothDeltaR, result.RightFoot.TargetRotation, RightPoleTarget, up );

		// PostAnimationUpdate is [Obsolete] with no replacement documented in the shipped XML
		// docs; kept defensively, matching TwoBoneIK/LookAtIK's already-verified usage.
#pragma warning disable CS0612
		Renderer.PostAnimationUpdate();
#pragma warning restore CS0612
	}

	// Plant mode (opt-in via PlantToGround): correct each foot's hover WITHOUT moving the pelvis.
	// Each foot's plant point (the ball bone, or the leg end when unset) is measured against the
	// ground; its trailing-minimum height over PlantWindowSeconds is the planted level, and the
	// foot is lowered by exactly that hover (down to PlantHeightOffset) with its authored rotation
	// kept. Authored step lifts and heel raises above the planted level ride on top untouched;
	// flat-authored feet come out flat; a grounded foot is never raised. The pelvis is left alone:
	// a shared full-body offset belongs on the model root, not here.
	private void SolvePlant()
	{
		_plantWindowL ??= new PlantWindow( PlantWindowSeconds );
		_plantWindowR ??= new PlantWindow( PlantWindowSeconds );
		_plantWindowL.WindowSeconds = PlantWindowSeconds;
		_plantWindowR.WindowSeconds = PlantWindowSeconds;

		Vector3 up = Vector3.Up;
		Vector3 origin = Renderer.WorldPosition;
		float now = Time.Now;
		float dt = Time.Delta;

		Renderer.TryGetBoneTransformAnimation( in _leftRoot, out var leftRootTx );
		Renderer.TryGetBoneTransformAnimation( in _leftMid, out var leftMidTx );
		Renderer.TryGetBoneTransformAnimation( in _leftEnd, out var leftEndTx );
		Renderer.TryGetBoneTransformAnimation( in _rightRoot, out var rightRootTx );
		Renderer.TryGetBoneTransformAnimation( in _rightMid, out var rightMidTx );
		Renderer.TryGetBoneTransformAnimation( in _rightEnd, out var rightEndTx );

		var leftPlantBone = _leftPlantResolved ? _leftPlant : _leftEnd;
		var rightPlantBone = _rightPlantResolved ? _rightPlant : _rightEnd;
		Renderer.TryGetBoneTransformAnimation( in leftPlantBone, out var leftPlantTx );
		Renderer.TryGetBoneTransformAnimation( in rightPlantBone, out var rightPlantTx );

		_smoothPlantL = SolvePlantCorrection( leftPlantTx.Position, up, origin, now, dt, _plantWindowL, _smoothPlantL,
			out bool leftGrounded, out float leftResidual );
		_smoothPlantR = SolvePlantCorrection( rightPlantTx.Position, up, origin, now, dt, _plantWindowR, _smoothPlantR,
			out bool rightGrounded, out float rightResidual );

		LeftGrounded = leftGrounded;
		RightGrounded = rightGrounded;
		LastPlantResidualL = leftResidual;
		LastPlantResidualR = rightResidual;
		CurrentPlantCorrectionL = _smoothPlantL;
		CurrentPlantCorrectionR = _smoothPlantR;
		CurrentPelvisOffset = 0f;

		// A settled zero correction skips the leg write entirely (the target IS the animated pose,
		// so the write is an unforced no-op - the same hazard the terrain path guards against).
		if ( _smoothPlantL != 0f )
			SolveLeg( in _leftRoot, in _leftMid, in _leftEnd, leftRootTx, leftMidTx, leftEndTx, _leftBindPose,
				Vector3.Zero, -_smoothPlantL, leftEndTx.Rotation.ToNumerics(), LeftPoleTarget, up );
		if ( _smoothPlantR != 0f )
			SolveLeg( in _rightRoot, in _rightMid, in _rightEnd, rightRootTx, rightMidTx, rightEndTx, _rightBindPose,
				Vector3.Zero, -_smoothPlantR, rightEndTx.Rotation.ToNumerics(), RightPoleTarget, up );

#pragma warning disable CS0612
		Renderer.PostAnimationUpdate();
#pragma warning restore CS0612
	}

	// One foot's smoothed downward plant correction (world units, positive = foot lowered). Reads
	// the plant point against the ground (a flat plane or a downward trace), pushes its height into
	// the trailing-minimum window, removes the trailing-minimum hover down to PlantHeightOffset,
	// and smooths the result. Returns 0 when no ground reference is available (nothing to plant to).
	private float SolvePlantCorrection( Vector3 plantPos, Vector3 up, Vector3 origin, float now, float dt,
		PlantWindow window, float smoothPrev, out bool grounded, out float residual )
	{
		float groundLevel;
		if ( UseFlatGround )
		{
			grounded = true;
			groundLevel = FlatGroundHeight;
		}
		else
		{
			var tr = TraceFoot( plantPos, up, origin );
			grounded = tr.Hit;
			groundLevel = tr.Hit ? Vector3.Dot( tr.HitPosition, up ) : 0f;
		}

		// Height of the plant point above the ground along the up axis (the hover to remove).
		residual = Vector3.Dot( plantPos, up ) - groundLevel;

		float target = 0f;
		if ( grounded )
		{
			float trailingMin = window.Push( now, residual );
			target = PlantToGroundSolver.ComputeCorrection( trailingMin, PlantHeightOffset, PlantMaxCorrection );
		}

		float smoothed = FootPlacementSolver.SmoothOffset( smoothPrev, target, SmoothingRate, dt );
		if ( target == 0f && MathF.Abs( smoothed ) < 1e-3f )
			smoothed = 0f;
		return smoothed;
	}

	private void SolveLeg( in BoneCollection.Bone rootBone, in BoneCollection.Bone midBone, in BoneCollection.Bone endBone,
		global::Transform rootTx, global::Transform midTx, global::Transform endTx, BindPoseData bindPose,
		Vector3 pelvisShift, float footDelta, System.Numerics.Quaternion footTargetRotation, GameObject? poleTarget, Vector3 up )
	{
		Vector3 shiftedRoot = rootTx.Position + pelvisShift;
		Vector3 shiftedMid = midTx.Position + pelvisShift;
		Vector3 shiftedEnd = endTx.Position + pelvisShift;

		Vector3 poleHint = poleTarget is not null && poleTarget.IsValid
			? poleTarget.WorldPosition - shiftedRoot
			: rootTx.Rotation * bindPose.DefaultPoleDirection.ToSandbox();

		var input = new TwoBoneIkInput
		{
			RootPosition = shiftedRoot.ToNumerics(),
			MidPosition = shiftedMid.ToNumerics(),
			EndPosition = shiftedEnd.ToNumerics(),
			RootRotation = rootTx.Rotation.ToNumerics(),
			MidRotation = midTx.Rotation.ToNumerics(),
			EndRotation = endTx.Rotation.ToNumerics(),
			TargetPosition = (endTx.Position + up * footDelta).ToNumerics(),
			TargetRotation = footTargetRotation,
			HasPole = true,
			PoleHint = poleHint.ToNumerics(),
			PoleAngleOffsetRadians = 0f,
			FallbackBendNormal = bindPose.BendNormal,
			PositionWeight = 1f,
			RotationWeight = FootRotationWeight,
			MasterWeight = Weight,
			SoftFraction = 0f,
			MaxStretch = 0f,
		};

		var result = TwoBoneIkSolver.Solve( input );

		// SetBoneTransform expects model-local space, unlike TryGetBoneTransformAnimation/
		// TryGetBoneTransform which are documented as worldspace - see MathBridge.ToModelLocal.
		Renderer!.SetBoneTransform( in rootBone, Renderer.ToModelLocal( new global::Transform( shiftedRoot, result.RootRotation.ToSandbox() ).WithScale( rootTx.Scale ) ) );
		Renderer.SetBoneTransform( in midBone, Renderer.ToModelLocal( new global::Transform( result.MidPosition.ToSandbox(), result.MidRotation.ToSandbox() ).WithScale( midTx.Scale ) ) );
		Renderer.SetBoneTransform( in endBone, Renderer.ToModelLocal( new global::Transform( result.EndPosition.ToSandbox(), result.EndRotation.ToSandbox() ).WithScale( endTx.Scale ) ) );
	}

	private (bool Hit, Vector3 HitPosition, Vector3 Normal) TraceFoot( Vector3 footPos, Vector3 up, Vector3 origin )
	{
		const float traceMargin = 2f;
		Vector3 footOnPlane = footPos - up * Vector3.Dot( footPos - origin, up );
		Vector3 traceStart = footOnPlane + up * (MaxStepUp + traceMargin);
		Vector3 traceEnd = footOnPlane - up * (MaxStepDown + traceMargin);

		var trace = Scene.Trace.FromTo( traceStart, traceEnd ).IgnoreGameObjectHierarchy( Renderer!.GameObject.Root );

		var tags = IgnoreTags.Split( ' ', StringSplitOptions.RemoveEmptyEntries );
		if ( tags.Length > 0 )
			trace = trace.WithoutTags( tags );

		var tr = trace.Run();
		return (tr.Hit, tr.HitPosition, tr.Normal);
	}

	[MemberNotNullWhen( true, nameof( Renderer ) )]
	private bool EnsureResolved()
	{
		if ( Renderer is null || Renderer.Model is null || string.IsNullOrEmpty( LeftFootBone ) || string.IsNullOrEmpty( RightFootBone ) )
			return false;

		var signature = (Renderer, LeftFootBone, RightFootBone, LeftRootOverride, LeftMidOverride, RightRootOverride, RightMidOverride, PelvisBoneOverride, LeftPlantBone, RightPlantBone);
		if ( signature.Equals( _cachedSignature ) && _leftRoot is not null )
			return true;

		var bones = Renderer.Model.Bones;

		if ( !TryResolveLeg( bones, LeftFootBone, LeftRootOverride, LeftMidOverride, out var leftRootNode, out _leftRoot, out _leftMid, out _leftEnd, out _leftBindPose ) )
			return false;

		if ( !TryResolveLeg( bones, RightFootBone, RightRootOverride, RightMidOverride, out var rightRootNode, out _rightRoot, out _rightMid, out _rightEnd, out _rightBindPose ) )
			return false;

		ResolvePelvis( bones, leftRootNode, rightRootNode );
		ResolvePlantBones( bones );

		_cachedSignature = signature;
		_smoothPelvis = 0f;
		_smoothDeltaL = 0f;
		_smoothDeltaR = 0f;
		_smoothPlantL = 0f;
		_smoothPlantR = 0f;
		_plantWindowL?.Reset();
		_plantWindowR?.Reset();

		return true;
	}

	// Plant points (ball / toe bone) whose ground contact is planted. Optional: an unset or
	// absent name falls back to the leg's own end bone at solve time (_*PlantResolved = false).
	private void ResolvePlantBones( BoneCollection bones )
	{
		_leftPlantResolved = false;
		_rightPlantResolved = false;

		if ( !string.IsNullOrEmpty( LeftPlantBone ) && bones.HasBone( LeftPlantBone ) )
		{
			_leftPlant = bones.GetBone( LeftPlantBone );
			_leftPlantResolved = true;
		}

		if ( !string.IsNullOrEmpty( RightPlantBone ) && bones.HasBone( RightPlantBone ) )
		{
			_rightPlant = bones.GetBone( RightPlantBone );
			_rightPlantResolved = true;
		}
	}

	private static bool TryResolveLeg( BoneCollection bones, string endBoneName, string rootOverrideName, string midOverrideName,
		out IBoneNode? rootNode, out BoneCollection.Bone rootBone, out BoneCollection.Bone midBone, out BoneCollection.Bone endBone,
		out BindPoseData bindPose )
	{
		rootNode = null;
		rootBone = null!;
		midBone = null!;
		endBone = null!;
		bindPose = default;

		if ( !bones.HasBone( endBoneName ) )
			return false;

		IBoneNode endNode = new SandboxBoneNode( bones.GetBone( endBoneName ) );

		IBoneNode? rootOverrideNode = null;
		if ( !string.IsNullOrEmpty( rootOverrideName ) )
		{
			if ( !bones.HasBone( rootOverrideName ) )
				return false;
			rootOverrideNode = new SandboxBoneNode( bones.GetBone( rootOverrideName ) );
		}

		IBoneNode? midOverrideNode = null;
		if ( !string.IsNullOrEmpty( midOverrideName ) )
		{
			if ( !bones.HasBone( midOverrideName ) )
				return false;
			midOverrideNode = new SandboxBoneNode( bones.GetBone( midOverrideName ) );
		}

		var chain = BoneChainResolver.Resolve( endNode, rootOverrideNode, midOverrideNode );
		if ( !chain.Success )
			return false;

		rootNode = chain.Root;
		rootBone = ((SandboxBoneNode)chain.Root!).Bone;
		midBone = ((SandboxBoneNode)chain.Mid!).Bone;
		endBone = ((SandboxBoneNode)chain.End!).Bone;

		var rootBindWorld = global::Transform.Zero;
		var midBindWorld = global::Transform.Concat( rootBindWorld, midBone.LocalTransform );
		var endBindWorld = global::Transform.Concat( midBindWorld, endBone.LocalTransform );

		bindPose = TwoBoneIkSolver.AnalyzeBindPose(
			rootBindWorld.Position.ToNumerics(),
			midBindWorld.Position.ToNumerics(),
			endBindWorld.Position.ToNumerics() );

		return true;
	}

	private void ResolvePelvis( BoneCollection bones, IBoneNode? leftRootNode, IBoneNode? rightRootNode )
	{
		if ( !string.IsNullOrEmpty( PelvisBoneOverride ) )
		{
			if ( bones.HasBone( PelvisBoneOverride ) )
			{
				var pelvisBone = bones.GetBone( PelvisBoneOverride );
				if ( !IsChainBone( pelvisBone.Name ) )
				{
					_pelvisBone = pelvisBone;
					PelvisResolved = true;
					return;
				}
			}

			PelvisResolved = false;
			return;
		}

		if ( leftRootNode is not null && rightRootNode is not null )
		{
			var ancestor = BoneChainResolver.FindCommonAncestor( leftRootNode, rightRootNode );
			if ( ancestor is not null && !IsChainBone( ancestor.Name ) && bones.HasBone( ancestor.Name ) )
			{
				_pelvisBone = bones.GetBone( ancestor.Name );
				PelvisResolved = true;
				return;
			}
		}

		PelvisResolved = false;
	}

	private bool IsChainBone( string name )
		=> name == _leftRoot.Name || name == _leftMid.Name || name == _leftEnd.Name
		|| name == _rightRoot.Name || name == _rightMid.Name || name == _rightEnd.Name;

	protected override void DrawGizmos()
	{
		if ( !EnsureResolved() )
		{
			Gizmo.Draw.Color = Color.Red;
			Gizmo.Draw.WorldText( "FootPlacementIK: invalid leg chains", new global::Transform( WorldPosition ) );
			return;
		}

		Renderer.TryGetBoneTransformAnimation( in _leftEnd, out var leftEndTx );
		Renderer.TryGetBoneTransformAnimation( in _rightEnd, out var rightEndTx );

		Vector3 up = Vector3.Up;
		Vector3 origin = Renderer.WorldPosition;

		DrawFootGizmo( leftEndTx.Position, up, origin, _smoothDeltaL );
		DrawFootGizmo( rightEndTx.Position, up, origin, _smoothDeltaR );

		if ( PelvisResolved )
		{
			Renderer.TryGetBoneTransformAnimation( in _pelvisBone, out var pelvisTx );
			if ( MathF.Abs( _smoothPelvis ) > 0.01f )
			{
				Gizmo.Draw.Color = Color.Magenta;
				Gizmo.Draw.Arrow( pelvisTx.Position, pelvisTx.Position + up * _smoothPelvis, 2f, 1f );
			}
		}
		else
		{
			Gizmo.Draw.Color = Color.Orange;
			Gizmo.Draw.WorldText( "FootPlacementIK: no pelvis resolved, feet-only mode (see PelvisBoneOverride)", new global::Transform( WorldPosition + Vector3.Up * 8f ) );
		}
	}

	private void DrawFootGizmo( Vector3 footPos, Vector3 up, Vector3 origin, float smoothDelta )
	{
		var trace = TraceFoot( footPos, up, origin );

		const float traceMargin = 2f;
		Vector3 footOnPlane = footPos - up * Vector3.Dot( footPos - origin, up );
		Vector3 traceStart = footOnPlane + up * (MaxStepUp + traceMargin);
		Vector3 traceEnd = footOnPlane - up * (MaxStepDown + traceMargin);

		Gizmo.Draw.Color = trace.Hit ? Color.Green : Color.Red;
		Gizmo.Draw.Line( traceStart, traceEnd );

		if ( trace.Hit )
		{
			Gizmo.Draw.LineSphere( new Sphere( trace.HitPosition, 1.5f ) );
			Gizmo.Draw.Arrow( trace.HitPosition, trace.HitPosition + trace.Normal * 6f, 1.5f, 1f );

			Gizmo.Draw.Color = Color.Cyan;
			Gizmo.Draw.LineSphere( new Sphere( footPos + up * smoothDelta, 1.5f ) );
		}
	}
}
