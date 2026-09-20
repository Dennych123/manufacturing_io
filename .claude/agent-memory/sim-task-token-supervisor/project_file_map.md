---
name: project-file-map
description: Which module owns what in manufacturing_io, so reads can be targeted instead of scanning the tree.
metadata:
  type: project
---

Repo layout and ownership (from docs/PLAN.md §12, verified present):

- `docs/PLAN.md` — the plan, 622 lines. Phases P0–P7, invariants, test matrix. Load once per session.
- `CLAUDE.md` — silent-failure rules (design, OPC UA/Sysmac, IO timing, browser, hygiene). Always in context via system reminder; do not re-read.
- `lib/` — shared by Node + browser, NEVER imports three/Rapier/node:. `math.js` (compose/invert, rot X-Y-Z deg), `scene.js` (owns `worldPoses()` + `validate()`), `components.js` (TYPES map), `analysis.js`.
- `server/` — `plant.js` (Rapier world, IO exchange, minPulseMs, recording), `opcua.js` (THE single OPC UA copy: browse+browseNext, batched write, twin read-only), `main.js` (HTTP/SSE/CLI).
- `web/` — `app.js` (viewer + IO panel), `editor.js` (P2 builder), `charts.js`. Poses come from `worldPoses()` only; no pose math here.
- `tools/gen_sysmac.js` — scene → Sysmac XML/iolist/probe/shim. Exports `sceneProject`, `externalVars`, `stText`, `PROBE`, `programXml`, `globalsXml`.
- `tools/smc2.js` (P5, untracked as of 2026-09-10) — writes globals/program/task straight into a CLOSED `.smc2` container. Reuses `externalVars`/`stText` from gen_sysmac (single copy). Resolves solution id via manifest (Studio changes it every Save).
- `tools/zip.js` (untracked) — `readZip/writeZip/content/makeEntry`, byte-exact ZIP round trip.
- `scenes/` — `<name>.json` (single source of numbers/names), optional `.st`, `.ctl.js`, generated `.sysmac.xml`, `.io.tsv`.
- `tests/run.js` — no-framework harness, `chk()` per suite, loud SKIP. Suites: lib, plant, opcua, sysmac, analysis, web, + smc2.test.js. 8 suites pass as of 2026-09-10.

**Why:** avoids re-scanning the tree each session.
**How to apply:** target the owning file for a question; grep before full reads.
