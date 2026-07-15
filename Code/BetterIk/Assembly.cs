#nullable enable annotations

// Global usings for the s&box in-engine compiler.
//
// The plain net8.0 dev harness gets these automatically via <ImplicitUsings>,
// but s&box's compiler injects no BCL usings at all - without this file the
// library fails to compile inside the editor.

global using System;
global using System.Collections.Generic;
global using System.Linq;

// NOTE on Vector3/Quaternion: s&box declares its own Vector3 and Rotation in the
// global namespace, which wins over `using System.Numerics;` during name lookup.
// The engine-agnostic math in Maths/ uses System.Numerics types exclusively so it
// also compiles in the plain net8.0 dev harness (dev/BetterIk.Dev.csproj). Every
// file under Maths/ that uses the simple name Vector3 or Quaternion carries a
// namespace-scoped alias after its file-scoped namespace line, e.g.:
//
//     namespace BetterIk.Maths;
//     using Vector3 = System.Numerics.Vector3;
//
// A global alias does not work here (CS0576: conflicts with the global-namespace
// type at every use site), the alias must be namespace-scoped per file.
