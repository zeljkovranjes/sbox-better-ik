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
public sealed class FootPlacementIK : Component
{
	[Property] public SkinnedModelRenderer? Renderer { get; set; }
	[Property] public string LeftFootBone { get; set; } = "";
	[Property] public string RightFootBone { get; set; } = "";

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

	[Property, Group( "Advanced" )] public string PelvisBoneOverride { get; set; } = "";
	[Property, Group( "Advanced" )] public GameObject? LeftPoleTarget { get; set; }
	[Property, Group( "Advanced" )] public GameObject? RightPoleTarget { get; set; }
	[Property, Group( "Advanced" )] public string LeftRootOverride { get; set; } = "";
	[Property, Group( "Advanced" )] public string LeftMidOverride { get; set; } = "";
	[Property, Group( "Advanced" )] public string RightRootOverride { get; set; } = "";
	[Property, Group( "Advanced" )] public string RightMidOverride { get; set; } = "";

	public bool HasValidChains { get; private set; }
	public bool PelvisResolved { get; private set; }
	public bool LeftGrounded { get; private set; }
	public bool RightGrounded { get; private set; }
	public float CurrentPelvisOffset { get; private set; }

	private (SkinnedModelRenderer Renderer, string LeftFootBone, string RightFootBone, string LeftRootOverride, string LeftMidOverride, string RightRootOverride, string RightMidOverride, string PelvisBoneOverride) _cachedSignature;

	// Only valid once EnsureResolved() has returned true at least once.
	private BoneCollection.Bone _leftRoot = null!;
	private BoneCollection.Bone _leftMid = null!;
	private BoneCollection.Bone _leftEnd = null!;
	private BoneCollection.Bone _rightRoot = null!;
	private BoneCollection.Bone _rightMid = null!;
	private BoneCollection.Bone _rightEnd = null!;
	private BoneCollection.Bone _pelvisBone = null!;

	private BindPoseData _leftBindPose;
	private BindPoseData _rightBindPose;

	private float _smoothPelvis;
	private float _smoothDeltaL;
	private float _smoothDeltaR;

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

		// Weight <= 0 skips the entire read-trace-solve-write cycle - see DECISIONS.md's
		// no-unforced-writes principle: any SetBoneTransform whose value has no restoring force
		// (not animated, not converging toward a fixed target) is not bit-lossless at the engine
		// write boundary and compounds to Infinity over enough frames (confirmed via FabrikIK).
		// Zero the smoothed state so re-enabling Weight later doesn't replay stale offsets.
		if ( Weight <= 0f )
		{
			_smoothPelvis = 0f;
			_smoothDeltaL = 0f;
			_smoothDeltaR = 0f;
			LeftGrounded = false;
			RightGrounded = false;
			CurrentPelvisOffset = 0f;
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

		var signature = (Renderer, LeftFootBone, RightFootBone, LeftRootOverride, LeftMidOverride, RightRootOverride, RightMidOverride, PelvisBoneOverride);
		if ( signature.Equals( _cachedSignature ) && _leftRoot is not null )
			return true;

		var bones = Renderer.Model.Bones;

		if ( !TryResolveLeg( bones, LeftFootBone, LeftRootOverride, LeftMidOverride, out var leftRootNode, out _leftRoot, out _leftMid, out _leftEnd, out _leftBindPose ) )
			return false;

		if ( !TryResolveLeg( bones, RightFootBone, RightRootOverride, RightMidOverride, out var rightRootNode, out _rightRoot, out _rightMid, out _rightEnd, out _rightBindPose ) )
			return false;

		ResolvePelvis( bones, leftRootNode, rightRootNode );

		_cachedSignature = signature;
		_smoothPelvis = 0f;
		_smoothDeltaL = 0f;
		_smoothDeltaR = 0f;

		return true;
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

		Gizmo.Draw.Color = Color.White;
		Gizmo.Draw.WorldText( $"W:{Weight:F2} Pelvis:{CurrentPelvisOffset:F1} L:{(LeftGrounded ? "G" : "-")} R:{(RightGrounded ? "G" : "-")}", new global::Transform( WorldPosition + Vector3.Up * 4f ) );
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
