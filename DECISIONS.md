# Decisions

Log of judgment calls made where the spec was silent, with reasoning. Newest entries at the bottom.

| Date | Decision | Reasoning |
|------|----------|-----------|
| 2026-07-15 | Ident `better_ik`, Org `notpointless`, matching sibling projects (humanoid-retargeter, monster-ai, the_director_ai, two_brains_ai) | Consistency with the author's existing s&box package namespace. |
| 2026-07-15 | Engine-agnostic solver math lives in `Code/BetterIk/Maths/`, using `System.Numerics.Vector3`/`Quaternion` with namespace-scoped aliases (not Sandbox's global `Vector3`/`Rotation`) | Mirrors the proven pattern in humanoid-retargeter: lets the same source compile both inside the s&box editor and in a plain net8.0 `dev/BetterIk.Dev.csproj`, so the analytic solver and pole vector math are unit-testable (xunit) with no editor running at all. This is the only way to get truly autonomous, no-human-in-the-loop testing for the core geometry, since the s&box MCP server requires a live editor process. |
