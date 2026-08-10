#nullable enable

using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Sandbox;
using BetterIk.Maths;
using BetterIk.Skeleton;

namespace BetterIk;

/// <summary>
/// Unconstrained FABRIK for variable-length chains (tails, tentacles, ropes). No pole vector, no
/// joint limits, no per-joint stiffness - for hinge-like limb IK, use TwoBoneIK instead. Drop on
/// the model root (or any descendant), set RootBone and EndBone to the chain's two ends; unlike
/// TwoBoneIK this cannot auto-walk a fixed depth, since a FABRIK chain's length is arbitrary, so
/// RootBone must be named explicitly.
/// </summary>
public sealed class FabrikIK : Component, IHasSkinnedRenderer
{
	[Property] public SkinnedModelRenderer? Renderer { get; set; }
	[Property, BoneName] public string RootBone { get; set; } = "";
	[Property, BoneName] public string EndBone { get; set; } = "";
	[Property] public GameObject? Target { get; set; }

	[Property, Range( 0f, 1f )] public float Weight { get; set; } = 1f;

	[Property, Group( "Advanced" )] public int MaxIterations { get; set; } = 16;

	// 0 = auto: 0.001 * total chain length, computed at solve time from the animated pose.
	[Property, Group( "Advanced" )] public float Tolerance { get; set; } = 0f;
	/// <summary>Read the final post-override pose (TryGetBoneTransform) instead of the raw
	/// animation pose. Enable ONLY when a full-pose driver (e.g. motion matching) clears and
	/// rewrites every bone every frame - on such characters the animation pose is stale bind-pose
	/// data. Leave OFF for animgraph-driven characters: with nothing rewriting bones each frame,
	/// this component would read back its own previous write and compound toward infinity.</summary>
	[Property, Group( "Advanced" )] public bool UseFinalPose { get; set; } = false;

	public bool HasValidChain { get; private set; }
	public bool LastSolveApplied { get; private set; }

	private (SkinnedModelRenderer Renderer, string RootBone, string EndBone) _cachedSignature;
	// Root-first, end-last. Only valid once EnsureResolved() has returned true at least once.
	private BoneCollection.Bone[] _chainBones = null!;
	private Vector3[] _chainScales = null!;

	protected override void OnStart()
	{
		Renderer ??= GameObject.GetComponent<SkinnedModelRenderer>()
			?? GameObject.GetComponentInParent<SkinnedModelRenderer>( includeSelf: true );
	}

	protected override void OnPreRender()
	{
		Solve();
	}

	// Overrides persist on the renderer until explicitly cleared, so disabling mid-play would
	// otherwise freeze the skeleton at its last IK'd pose forever on any character where nothing
	// else rewrites bones every frame.
	protected override void OnDisabled() => Renderer?.ClearPhysicsBones();

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
		// pose and writing the identical value back. Live testing found the latter is NOT
		// perfectly lossless through the world<->model-local round trip (Renderer.ToModelLocal)
		// every frame: on Rig B's tail, repeatedly reading TryGetBoneTransformAnimation then
		// writing the visually-unchanged result back via SetBoneTransform compounded into
		// Infinity within a few seconds, even with bone scale frozen (Renderer.ToModelLocal
		// finding above). Not writing at all when there is nothing to blend is strictly safer.
		if ( Weight <= 0f )
		{
			LastSolveApplied = false;
			return;
		}

		int n = _chainBones.Length;
		var positions = new System.Numerics.Vector3[n];
		var rotations = new System.Numerics.Quaternion[n];

		float totalLength = 0f;
		for ( int i = 0; i < n; i++ )
		{
			Renderer.TryGetBonePose( in _chainBones[i], UseFinalPose, out var tx );
			positions[i] = tx.Position.ToNumerics();
			rotations[i] = tx.Rotation.ToNumerics();
			if ( i > 0 )
				totalLength += (positions[i] - positions[i - 1]).Length();
		}

		float tolerance = Tolerance > 0f ? Tolerance : 0.001f * MathF.Max( totalLength, 1e-4f );

		var input = new FabrikInput
		{
			JointPositions = positions,
			TargetPosition = Target.WorldPosition.ToNumerics(),
			Weight = Weight,
			MaxIterations = MaxIterations,
			Tolerance = tolerance,
		};

		var result = FabrikSolver.Solve( input );
		var solvedRotations = FabrikSolver.DeriveRotations( positions, result.JointPositions, rotations );

		for ( int i = 0; i < n; i++ )
		{
			Renderer.SetBoneTransform( in _chainBones[i], Renderer.ToModelLocal(
				new global::Transform( result.JointPositions[i].ToSandbox(), solvedRotations[i].ToSandbox() ).WithScale( _chainScales[i] ) ) );
		}

		// PostAnimationUpdate is [Obsolete] with no replacement documented in the shipped XML
		// docs; kept defensively, matching the other components' already-verified usage.
#pragma warning disable CS0612
		Renderer.PostAnimationUpdate();
#pragma warning restore CS0612

		LastSolveApplied = true;
	}

	[MemberNotNullWhen( true, nameof( Renderer ) )]
	private bool EnsureResolved()
	{
		if ( Renderer is null || Renderer.Model is null || string.IsNullOrEmpty( RootBone ) || string.IsNullOrEmpty( EndBone ) )
			return false;

		var signature = (Renderer, RootBone, EndBone);
		if ( signature.Equals( _cachedSignature ) && _chainBones is not null )
			return true;

		var bones = Renderer.Model.Bones;
		if ( !bones.HasBone( EndBone ) )
			return false;

		IBoneNode endNode = new SandboxBoneNode( bones.GetBone( EndBone ) );
		var chain = BoneChainResolver.ResolveNamedChain( endNode, RootBone );
		if ( !chain.Success )
			return false;

		_chainBones = chain.Chain!.Select( node => ((SandboxBoneNode)node).Bone ).ToArray();
		_cachedSignature = signature;

		// Cached once here rather than re-read from TryGetBoneTransformAnimation every frame.
		// Re-reading and re-writing world scale every frame round-trips it through
		// Renderer.ToModelLocal on every solve; live testing found this compounds into Infinity
		// within a few seconds on Rig B's tail (bone scale is not exactly 1 on that rig). IK never
		// needs to change a bone's scale, so freezing it at first resolution is both safe and
		// removes the feedback path entirely.
		_chainScales = new Vector3[_chainBones.Length];
		for ( int i = 0; i < _chainBones.Length; i++ )
		{
			Renderer.TryGetBoneTransformAnimation( in _chainBones[i], out var tx );
			_chainScales[i] = tx.Scale;
		}

		return true;
	}

	protected override void DrawGizmos()
	{
		if ( !EnsureResolved() )
		{
			Gizmo.Draw.Color = Color.Red;
			Gizmo.Draw.WorldText( "FabrikIK: invalid chain", new global::Transform( WorldPosition ) );
			return;
		}

		if ( Target is null || !Target.IsValid )
			return;

		Gizmo.Draw.Color = Color.Yellow;
		for ( int i = 0; i < _chainBones.Length - 1; i++ )
		{
			Renderer.TryGetBoneTransform( in _chainBones[i], out var a );
			Renderer.TryGetBoneTransform( in _chainBones[i + 1], out var b );
			Gizmo.Draw.Line( a.Position, b.Position );
		}

		Gizmo.Draw.Color = Color.White;
		Gizmo.Draw.LineSphere( new Sphere( Target.WorldPosition, 2f ) );
	}
}
