#nullable enable annotations

// Inspector picker for [BoneName]-tagged string properties (TwoBoneIK.EndBone,
// FootPlacementIK.LeftFootBone, etc.) - renders a dropdown of the resolved
// SkinnedModelRenderer's actual bone names instead of a plain text box. See
// BoneNameAttribute in Code/BetterIk for the marker attribute this hooks off of.

using System.Collections.Generic;
using System.Linq;
using Editor;
using Sandbox;
using BetterIk;

namespace BetterIk.Editor;

[CustomEditor( typeof( string ), WithAllAttributes = new[] { typeof( BoneNameAttribute ) } )]
public class BoneNameControlWidget : DropdownControlWidget<string>
{
	public BoneNameControlWidget( SerializedProperty property ) : base( property )
	{
	}

	protected override IEnumerable<object> GetDropdownValues()
	{
		var renderer = ResolveRenderer();
		if ( renderer?.Model is null )
			return Enumerable.Empty<object>();

		return renderer.Model.Bones.AllBones
			.Select( bone => (object)new Entry { Value = bone.Name, Label = bone.Name } );
	}

	protected override string GetDisplayText()
	{
		var value = SerializedProperty.GetValue<string>( null );
		return string.IsNullOrEmpty( value ) ? "None" : value;
	}

	private SkinnedModelRenderer ResolveRenderer()
	{
		var target = SerializedProperty.Parent?.Targets?.FirstOrDefault();
		if ( target is not IHasSkinnedRenderer hasRenderer )
			return null;

		if ( hasRenderer.Renderer is not null )
			return hasRenderer.Renderer;

		if ( target is not Component component )
			return null;

		return component.GameObject?.GetComponent<SkinnedModelRenderer>()
			?? component.GameObject?.GetComponentInParent<SkinnedModelRenderer>( includeSelf: true );
	}
}
