#nullable enable

using Sandbox;

namespace BetterIk;

/// <summary>
/// Drives RootOffset then OrientationWarp at Stage.UpdateBones, order 1000 - after animation
/// evaluation (order 0) and after any external full-pose driver (e.g. motion matching's own
/// system, which runs at order -1000), and before every IK component's own OnPreRender. GASP
/// ordering: base pose -> root offset -> orientation warp -> foot IK (FootPlacementIK stays driven
/// by its own OnPreRender, unchanged, so it re-plants feet after any root/orientation offset).
/// </summary>
public sealed class BetterIkSystem : GameObjectSystem
{
	public BetterIkSystem( Scene scene ) : base( scene )
	{
		Listen( Stage.UpdateBones, 1000, Apply, "BetterIk.Apply" );
	}

	private void Apply()
	{
		foreach ( RootOffset component in Scene.GetAllComponents<RootOffset>() )
		{
			if ( component.Active )
				component.Apply();
		}

		foreach ( OrientationWarp component in Scene.GetAllComponents<OrientationWarp>() )
		{
			if ( component.Active )
				component.Apply();
		}
	}
}
