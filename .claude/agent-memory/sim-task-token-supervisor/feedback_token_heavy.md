---
name: feedback-token-heavy
description: Which files/ops to grep instead of loading wholesale in manufacturing_io.
metadata:
  type: feedback
---

Cheapest-first read strategy for this repo.

**Why:** context budget; several files are large or repetitive.
**How to apply:**
- `docs/PLAN.md` — 622 lines. Load once per session, then reference by section number (§3 components, §5 IO timing, §13 phases, §14 tests). Do not re-read.
- `CLAUDE.md` — already injected via system reminder every turn. Never read it with the Read tool.
- `tests/run.js` output — run `node tests/run.js 2>&1 | tail -40` for pass/fail; do not read suite source unless a test fails.
- Invariant checks on tools — grep for the trap tokens (`P_`, `ATAN2`, `\r\n`, `hard-code`, `write(`, `worldPoses`) rather than reading the whole file. See [[feedback-invariant-scan]].
- `.smc2`/`.sysmac.xml` generated output — never read wholesale; use `--check` (byte-compare) and the sysmac test suite.
- git status snapshot in the briefing can be STALE — always run `git status --short` fresh.
