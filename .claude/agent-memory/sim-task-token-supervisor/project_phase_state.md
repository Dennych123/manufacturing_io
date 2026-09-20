---
name: project-phase-state
description: Phase progress and resume point for manufacturing_io as of last supervised session.
metadata:
  type: project
---

As of 2026-09-10, branch `p1-plant-viewer`:

- P0 (bootstrap, OPC UA port, latency): committed. minPulseMs default 20 ms measured.
- P1 (plant, viewer, cyl-on-slide, round trip): committed and passes live against Sysmac simulator (commit 59f5bf0). Exit met.
- **In flight (uncommitted, tested):** `tools/smc2.js` + `tools/zip.js` + `tests/smc2.test.js` — automates Studio manual Steps 1–2 (import + task assignment) by editing a CLOSED `.smc2` container. This is early P5 tooling. gen_sysmac.js refactored to share `externalVars`/`stText` (single copy). Docs updated: CLAUDE.md (3 new rules), README, SETUP.md, SMC2.md.
- All 8 test suites pass. XSD check SKIPs loudly (needs MIO_SYSMAC_REPO env / sysmac repo).

**Not yet proven:** Studio opening a tool-written `.smc2` and building it. Must test on a copy (`plc/test_mio_auto.smc2`) before trusting — this is the one open risk on the in-flight work.

**Next uncommitted decision:** whether to commit the smc2 tool now (Phase 1 branch) or move it onto a P5 branch. Ask user; the tool is P5 scope on a P1 branch.

**Why:** lets the next session resume without re-reading the diff.
**How to apply:** on resume, confirm branch + `git status`; the smc2 tool is the live work.
