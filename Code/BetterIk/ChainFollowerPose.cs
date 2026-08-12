#nullable enable

using Sandbox;

namespace BetterIk;

/// <summary>
/// Preserves twist, helper, and other skinned descendants when a solver replaces the three
/// primary transforms of a limb. Bone overrides are model-space values, so descendants that
/// were already overridden by another full-pose system do not automatically inherit a later
/// IK write to their parent.
/// </summary>
internal sealed class ChainFollowerPose
{
	private enum Anchor : byte
	{
		Root,
		Mid,
		End
	}

	private readonly record struct Binding( BoneCollection.Bone Bone, Anchor Anchor );
	private readonly record struct Sample( bool Valid, global::Transform Relative );

	private Binding[] _bindings = [];
	private Sample[] _samples = [];

	public void Resolve( BoneCollection bones, in BoneCollection.Bone root, in BoneCollection.Bone mid,
		in BoneCollection.Bone end )
	{
		var bindings = new List<Binding>();
		foreach ( var bone in bones.AllBones )
		{
			if ( bone.Index == root.Index || bone.Index == mid.Index || bone.Index == end.Index )
				continue;

			if ( TryFindAnchor( bone, root, mid, end, out var anchor ) )
				bindings.Add( new Binding( bone, anchor ) );
		}

		_bindings = bindings.ToArray();
		_samples = new Sample[_bindings.Length];
	}

	public void Capture( SkinnedModelRenderer renderer, bool useFinalPose, in global::Transform root,
		in global::Transform mid, in global::Transform end )
	{
		for ( var i = 0; i < _bindings.Length; i++ )
		{
			var binding = _bindings[i];
			var bone = binding.Bone;
			if ( !renderer.TryGetBonePose( in bone, useFinalPose, out var world ) )
			{
				_samples[i] = default;
				continue;
			}

			var anchor = SelectAnchor( binding.Anchor, root, mid, end );
			_samples[i] = new Sample( true, anchor.ToLocal( world ) );
		}
	}

	public void Write( SkinnedModelRenderer renderer, in global::Transform root, in global::Transform mid,
		in global::Transform end )
	{
		for ( var i = 0; i < _bindings.Length; i++ )
		{
			var sample = _samples[i];
			if ( !sample.Valid ) continue;
			var binding = _bindings[i];
			var anchor = SelectAnchor( binding.Anchor, root, mid, end );
			var modelSpace = renderer.ToModelLocal( anchor.ToWorld( sample.Relative ) );
			var bone = binding.Bone;
			renderer.SetBoneTransform( in bone, modelSpace );
		}
	}

	private static bool TryFindAnchor( BoneCollection.Bone bone, in BoneCollection.Bone root,
		in BoneCollection.Bone mid, in BoneCollection.Bone end, out Anchor anchor )
	{
		for ( var parent = bone.Parent; parent is not null; parent = parent.Parent )
		{
			if ( parent.Index == end.Index ) { anchor = Anchor.End; return true; }
			if ( parent.Index == mid.Index ) { anchor = Anchor.Mid; return true; }
			if ( parent.Index == root.Index ) { anchor = Anchor.Root; return true; }
		}

		anchor = default;
		return false;
	}

	private static global::Transform SelectAnchor( Anchor anchor, in global::Transform root,
		in global::Transform mid, in global::Transform end ) => anchor switch
	{
		Anchor.Root => root,
		Anchor.Mid => mid,
		_ => end
	};
}
