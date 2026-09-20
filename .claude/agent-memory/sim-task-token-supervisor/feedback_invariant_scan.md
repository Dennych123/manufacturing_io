---
name: feedback-invariant-scan
description: The cheap grep that checks manufacturing_io silent-failure invariants on a changed file.
metadata:
  type: feedback
---

To audit a changed/new file against the CLAUDE.md silent-failure rules without reading it wholesale, grep for the trap tokens:

`P_` (POU rename), `ATAN2` (not in W560), `\r\n`/CRLF vs LF (`.smc2`=CRLF, XML `<ST>`=LF), `write(` (twin must be read-only), `worldPoses` (poses one source), `three`/`Rapier`/`node:` (forbidden in lib/), `hard-code`/`solution` (solution id via manifest, changes every Save), `Float64Array`/`plain(` (typed array → Array), `setCanSleep`.

**Why:** these break silently; a targeted grep catches them for a fraction of a full read.
**How to apply:** run one Grep with the alternation over the changed file before approving a step. Confirmed effective on tools/smc2.js (2026-09-10): caught that it correctly reuses `externalVars`/`stText`, uses CRLF for zip + LF for XML, rejects `P_`, resolves solution via manifest.
