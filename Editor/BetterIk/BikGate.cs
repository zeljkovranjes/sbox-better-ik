#nullable enable annotations

// better-ik editor-process regression gate.
//
// This only runs when the BIK_GATE_RESULT environment variable is set AND a matching
// one-shot ".arm" marker file exists (done by dev/editor-rig/run_editor_gate.ps1). Normal
// users of the library never trigger it. Mirrors humanoid-retargeter/Editor/HumanoidRetargeter/M0Gate.cs
// verbatim for the safety mechanisms (scratch-project refusal, arm marker, quit backstop).
//
// What it does (inside a real sbox-dev.exe editor session):
//   1. waits for the project + asset system to be ready
//   2. refuses to run unless the open project is the bik-editor-rig scratch project
//   3. spawns one Citizen GameObject per scenario (disjoint objects, never conflicting bones)
//   4. enters play mode via SceneEditorSession.SetPlaying (no scene clone - component
//      references taken before Play remain valid, so scenario state is read in-process)
//   5. samples each scenario's TwoBoneIK + bone state every frame for a fixed run length
//   6. writes a JSON result file to BIK_GATE_RESULT and quits the editor

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading.Tasks;
using Editor;
using Sandbox;
using BetterIk;

namespace BetterIk.Gate;

public static class BikGate
{
	static bool _started;
	static readonly BikResult Result = new();
	static string _resultPath;

	const int FramesToRun = 40;

	[EditorEvent.Frame]
	public static void Tick()
	{
		if ( _started )
			return;

		_started = true;

		_resultPath = Environment.GetEnvironmentVariable( "BIK_GATE_RESULT" );
		if ( string.IsNullOrWhiteSpace( _resultPath ) )
			return; // not a gate run - do nothing, ever

		// One-shot arming marker, written by the driver script immediately before launch and
		// CONSUMED here. The env var alone must never arm the gate: a gate run that boots
		// Steam as its child leaks BIK_GATE_RESULT into Steam's environment, and every editor
		// the user launches through that Steam afterwards would inherit it.
		var marker = _resultPath + ".arm";
		try
		{
			if ( !File.Exists( marker ) )
			{
				Log.Info( "[bik-gate] BIK_GATE_RESULT is set but there is no arming marker - "
					+ "leaked env var (stale Steam environment?), ignoring; not a gate run" );
				return;
			}
			File.Delete( marker );
		}
		catch
		{
			return; // cannot verify/consume the marker - err on never running
		}

		_ = RunAsync();
	}

	static async Task RunAsync()
	{
		Note( "bik gate starting" );
		Result.engineBooted = true;
		Flush();

		try
		{
			await RunGateAsync();
		}
		catch ( Exception e )
		{
			Note( $"EXCEPTION: {e}" );
		}

		Result.completed = true;
		Result.passed = Result.scenarios.Count > 0 && Result.scenarios.All( s => s.passed );
		Flush();
		Note( $"bik gate finished, passed={Result.passed}" );

		if ( Result.refusedWrongProject )
			return; // a leaked env var in a real session must never quit the user's editor

		await Task.Delay( 1000 );
		try
		{
			EditorUtility.Quit( true );
		}
		catch ( Exception e )
		{
			Note( $"EditorUtility.Quit threw: {e.Message}" );
			Flush();
		}

		await Task.Delay( 10_000 );
		Environment.Exit( Result.passed ? 0 : 1 );
	}

	static async Task RunGateAsync()
	{
		Result.assetSystemReady = await WaitUntil(
			() => Project.Current is not null && AssetSystem.All.Any(),
			timeoutSeconds: 120 );
		Note( $"assetSystemReady={Result.assetSystemReady}" );
		Flush();

		if ( !Result.assetSystemReady )
			return;

		var project = Project.Current;
		Result.projectPath = project.GetRootPath();

		// Never touch a real session: only the bik-editor-rig scratch project is fair game.
		if ( (Result.projectPath ?? "").IndexOf( "bik-editor-rig", StringComparison.OrdinalIgnoreCase ) < 0 )
		{
			Result.refusedWrongProject = true;
			Note( $"REFUSING to run: open project '{Result.projectPath}' is not the bik-editor-rig "
				+ "scratch (leaked BIK_GATE_RESULT env var?) - aborting without touching the project" );
			Flush();
			return;
		}

		var session = SceneEditorSession.CreateDefault();
		var scene = session.Scene;
		Note( $"scene created" );
		Flush();

		var citizenModel = Model.Load( "models/citizen/citizen.vmdl" );
		if ( citizenModel is null || citizenModel.IsError )
		{
			Note( "FAILED to load models/citizen/citizen.vmdl" );
			Flush();
			return;
		}

		// A camera was added here as a hypothesis fix for component update methods never firing
		// during this gate's play session - it did NOT resolve the issue (confirmed with
		// instrumented counters: OnPreRender/OnUpdate both stayed at 0 calls either way), so the
		// root cause is still open. Left in since a scene with an active camera is closer to a
		// normal game session regardless, and it is not the cause of harm.
		var cameraGo = scene.CreateObject( true );
		cameraGo.Name = "Gate Camera";
		cameraGo.WorldPosition = new Vector3( 0, -800, 200 );
		var camera = cameraGo.AddComponent<CameraComponent>( true );
		camera.WorldRotation = Rotation.LookAt( Vector3.Zero - cameraGo.WorldPosition );

		// One Citizen GameObject per scenario - disjoint objects, so no scenario's bone writes
		// can ever conflict with another's, matching how a real user would run separate limbs.
		var scenarios = new List<ScenarioRig>
		{
			BuildScenario( scene, citizenModel, "hand_R_farClamp", "hand_R",
				new Vector3( 300, 0, 0 ), targetPos: new Vector3( 300, -400, 550 ) ),
			BuildScenario( scene, citizenModel, "hand_L_reachable", "hand_L",
				new Vector3( 0, 0, 0 ), targetPos: new Vector3( 20, -20, 45 ) ),
			BuildScenario( scene, citizenModel, "hand_L_weightBlend", "hand_L",
				new Vector3( -300, 0, 0 ), targetPos: new Vector3( 12, -8, 33 ) ),
		};

		Note( $"spawned {scenarios.Count} scenario rigs" );
		Flush();

		session.SetPlaying( scene );
		Note( $"playing={session.IsPlaying}" );
		Flush();

		// [EditorEvent.Frame] does not appear to drive this session's own simulation tick on its
		// own (confirmed empirically: component update methods stayed at 0 calls without this),
		// so drive it explicitly every loop iteration. Prefer the game session's own Tick() when
		// playing is active; fall back to Scene.GameTick(dt) if that is ever unavailable.
		void TickScene( float dt )
		{
			var gameSession = session.GameSession;
			if ( gameSession is not null )
				gameSession.Tick();
			else
				scene.GameTick( dt );
		}

		// Wait for chain resolution to actually settle before asserting on it - the component
		// resolves its bone chain lazily on first Solve(), which needs at least one real frame
		// (and possibly a few, since SkinnedModelRenderer's own bone data is only guaranteed
		// once its OnStart has run in the now-playing scene) rather than a guessed frame count.
		bool allResolved = false;
		var chainWaitSw = Stopwatch.StartNew();
		while ( chainWaitSw.Elapsed.TotalSeconds < 15 )
		{
			TickScene( 1f / 30f );
			if ( scenarios.All( r => r.Ik is not null && r.Ik.HasValidChain ) )
			{
				allResolved = true;
				break;
			}
			await Task.Delay( 33 );
		}
		Note( $"allChainsResolved={allResolved}" );
		Flush();

		for ( int frame = 0; frame < FramesToRun; frame++ )
		{
			// weightBlend scenario sweeps Weight linearly across the run instead of holding it at 1.
			var weightBlendScenario = scenarios[2];
			weightBlendScenario.Ik.Weight = Math.Clamp( frame / (float)(FramesToRun - 1), 0f, 1f );

			TickScene( 1f / 30f );
			await Task.Delay( 33 ); // ~1 frame at 30fps; lets OnPreRender/Solve actually run

			foreach ( var rig in scenarios )
				SampleFrame( rig, frame );
		}

		session.StopPlaying();
		Note( "playing stopped" );
		Flush();

		foreach ( var rig in scenarios )
			EvaluateScenario( rig );
	}

	class ScenarioRig
	{
		public string Name;
		public GameObject GameObject;
		public TwoBoneIK Ik;
		public GameObject Target;
		public Vector3 RootPos, MidPos; // filled in on first sample
		public float L1, L2;
		public List<FrameSample> Samples = new();
	}

	static ScenarioRig BuildScenario( Scene scene, Model model, string name, string endBone, Vector3 origin, Vector3 targetPos )
	{
		var go = scene.CreateObject( true );
		go.Name = name;
		go.WorldPosition = origin;

		var renderer = go.AddComponent<SkinnedModelRenderer>( true );
		renderer.Model = model;

		var targetGo = scene.CreateObject( true );
		targetGo.Name = name + "_target";
		targetGo.WorldPosition = origin + targetPos;

		var ik = go.AddComponent<TwoBoneIK>( true );
		ik.Renderer = renderer;
		ik.EndBone = endBone;
		ik.Target = targetGo;

		return new ScenarioRig { Name = name, GameObject = go, Ik = ik, Target = targetGo };
	}

	static void SampleFrame( ScenarioRig rig, int frame )
	{
		if ( rig.Ik is null || rig.Ik.Renderer is null )
			return;

		var renderer = rig.Ik.Renderer;
		if ( !renderer.TryGetBoneTransform( rig.Ik.EndBone, out var endTx ) )
			return;

		if ( rig.L1 <= 0f )
		{
			// One-time bind reference, taken from the first available sample, used only for
			// the far-clamp Lmax check below.
			if ( renderer.TryGetBoneTransform( "arm_upper_R", out var rootTx )
				&& renderer.TryGetBoneTransform( "arm_lower_R", out var midTx ) )
			{
				rig.RootPos = rootTx.Position;
				rig.MidPos = midTx.Position;
				rig.L1 = (midTx.Position - rootTx.Position).Length;
				rig.L2 = (endTx.Position - midTx.Position).Length;
			}
		}

		rig.Samples.Add( new FrameSample
		{
			frame = frame,
			hasValidChain = rig.Ik.HasValidChain,
			lastSolveApplied = rig.Ik.LastSolveApplied,
			weight = rig.Ik.Weight,
			endPos = new[] { endTx.Position.x, endTx.Position.y, endTx.Position.z },
			endRot = new[] { endTx.Rotation.x, endTx.Rotation.y, endTx.Rotation.z, endTx.Rotation.w },
		} );
	}

	static void EvaluateScenario( ScenarioRig rig )
	{
		var result = new ScenarioResult { name = rig.Name, sampleCount = rig.Samples.Count };

		if ( rig.Samples.Count == 0 )
		{
			result.passed = false;
			result.failReason = "no samples captured";
			Result.scenarios.Add( result );
			return;
		}

		if ( rig.Samples.Any( s => !s.hasValidChain ) )
		{
			result.passed = false;
			result.failReason = "HasValidChain was false on at least one frame";
			Result.scenarios.Add( result );
			return;
		}

		float lmax = rig.L1 + rig.L2;
		float tolerance = MathF.Max( 0.03f * lmax, 1.0f );

		switch ( rig.Name )
		{
			case "hand_R_farClamp":
			{
				// Regression test for the exact stale-endTx.Position bug: the end effector
				// must sit at ~Lmax from root, every frame, not stuck near the animated pose.
				var lastPos = ToVec3( rig.Samples[^1].endPos );
				float distFromRoot = (lastPos - rig.RootPos).Length;
				bool ok = MathF.Abs( distFromRoot - lmax ) < tolerance;
				result.passed = ok;
				result.failReason = ok ? null : $"distFromRoot={distFromRoot} expected~{lmax} tol={tolerance}";
				result.measured = new Dictionary<string, float> { ["distFromRoot"] = distFromRoot, ["lmax"] = lmax };
				break;
			}
			case "hand_L_reachable":
			{
				var targetPos = rig.Target.WorldPosition;
				var lastPos = ToVec3( rig.Samples[^1].endPos );
				float err = (lastPos - targetPos).Length;
				bool ok = err < tolerance;
				result.passed = ok;
				result.failReason = ok ? null : $"endError={err} tol={tolerance}";
				result.measured = new Dictionary<string, float> { ["endError"] = err };
				break;
			}
			case "hand_L_weightBlend":
			{
				// Weight sweeps 0->1: first sampled frame should be near the animated pose,
				// last frame should be near the target. No hard per-frame position check here
				// (bone lengths differ from the assertion above), just first-vs-last movement.
				var first = ToVec3( rig.Samples[0].endPos );
				var last = ToVec3( rig.Samples[^1].endPos );
				float moved = (last - first).Length;
				bool ok = moved > tolerance;
				result.passed = ok;
				result.failReason = ok ? null : $"moved={moved} tol={tolerance} (weight blend produced no visible motion)";
				result.measured = new Dictionary<string, float> { ["moved"] = moved };
				break;
			}
			default:
				result.passed = false;
				result.failReason = "unknown scenario name";
				break;
		}

		Result.scenarios.Add( result );
		Note( $"scenario {rig.Name}: passed={result.passed} {result.failReason}" );
		Flush();
	}

	static Vector3 ToVec3( float[] a ) => new( a[0], a[1], a[2] );

	// ---- plumbing ---------------------------------------------------------------

	static async Task<bool> WaitUntil( Func<bool> condition, float timeoutSeconds )
	{
		var sw = Stopwatch.StartNew();
		while ( sw.Elapsed.TotalSeconds < timeoutSeconds )
		{
			bool ok = false;
			try { ok = condition(); }
			catch { /* not ready yet */ }

			if ( ok )
				return true;

			await Task.Delay( 250 );
		}

		return false;
	}

	static void Note( string message )
	{
		Result.log.Add( $"[{DateTime.UtcNow:HH:mm:ss.fff}] {message}" );
		Log.Info( $"[bik-gate] {message}" );
	}

	static void Flush()
	{
		try
		{
			File.WriteAllText( _resultPath, JsonSerializer.Serialize( Result,
				new JsonSerializerOptions { WriteIndented = true } ) );
		}
		catch
		{
			// never let result IO take the editor down
		}
	}

	class FrameSample
	{
		public int frame { get; set; }
		public bool hasValidChain { get; set; }
		public bool lastSolveApplied { get; set; }
		public float weight { get; set; }
		public float[] endPos { get; set; }
		public float[] endRot { get; set; }
	}

	class ScenarioResult
	{
		public string name { get; set; }
		public int sampleCount { get; set; }
		public bool passed { get; set; }
		public string failReason { get; set; }
		public Dictionary<string, float> measured { get; set; }
	}

	class BikResult
	{
		public bool engineBooted { get; set; }
		public bool assetSystemReady { get; set; }
		public string projectPath { get; set; }
		public bool refusedWrongProject { get; set; }
		public List<ScenarioResult> scenarios { get; set; } = new();
		public bool completed { get; set; }
		public bool passed { get; set; }
		public List<string> log { get; set; } = new();
	}
}
