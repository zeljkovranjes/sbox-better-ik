#nullable enable

using Sandbox;

namespace BetterIk;

/// <summary>Implemented by every IK component that operates on a SkinnedModelRenderer, so editor
/// tooling (BoneNameControlWidget) can resolve the renderer to list bones without reflection.</summary>
public interface IHasSkinnedRenderer
{
	SkinnedModelRenderer? Renderer { get; }
}
