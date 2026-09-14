---
name: code-review
description: "Review a diff, a branch or a file in manufacturing_io against the rules that break things SILENTLY here: measured claims, trap+rule test pairs, .st/.ctl.js parity, regenerated artefacts, the plant/browser boundary. Use for 'review this', 'audit this diff', 'is this ready to commit'."
---

# /code-review

Review code in this repo the way its bugs actually arrive: silently. Nothing here crashes when a
rule is broken — the picture lies, a sensor is missed, a sequence waits for ever, or a soak says
OK about a machine that seized fifteen minutes earlier.

## Scope

Default: the uncommitted diff plus the last commit. `--branch` reviews `main..HEAD`, or name files.
Read [CLAUDE.md](../../../CLAUDE.md) first — it is the list of rules already paid for. Do not
re-derive them; check against them.

## Order of checks (stop wasting time on style)

1. **Claims without measurement.** Any comment, commit message or doc line stating a number, a
   cause or "this is faster/cheaper" must come from a run whose output exists. If the change says
   something is the bottleneck, ask where the measurement is. `compile()` per frame *looked* like
   the cost and measured 1 µs.
2. **Tests that pin only the fix.** A measured trap gets TWO checks: the trap (what goes wrong)
   and the rule (what avoids it). `tests/rapier.test.js` is the model. A test that would still
   pass with the bug restored is not a test.
3. **`.st` ↔ `.ctl.js` parity.** Every scene's ST program and its internal controller are the same
   sequence. A change to one without the other is a defect, even when both "work": the internal
   run and the PLC run then disagree, and only the PLC run is real.
4. **Generated files.** Changed a `.st` or a scene? `node tools/gen_sysmac.js --scenes` must have
   been re-run, or `--check` fails in CI and the committed XML lies. Same for any generator.
5. **Sequence discipline** (the expensive ones):
   - a waiting step waits for an INVARIANT, never "a counter moved";
   - an invariant must stay REACHABLE — anything that leaves without being counted needs a
     write-off, and the write-off belongs to the fault acknowledgement, not to every START;
   - every waiting step needs a watchdog. A machine that waits for ever looks alive and is not;
   - commands are levels or counters, never one-scan pulses; replies are `hold: false`.
6. **The boundaries.**
   - `lib/` imports neither three nor Rapier nor `node:`;
   - poses come from `worldPoses()` on both sides; no pose math in `web/`;
   - physics never calls `driver.write`, and never gates sequence or safety;
   - the plant never lies to the PLC to make a sequence work.
7. **Determinism.** New input paths (viewer edges, API calls) are QUEUED and applied at the start
   of a step, so a recorded run replays identically. Check `hands`/`presses` as the pattern. Any
   `Math.random` outside a seeded `rng()` is a defect.
8. **Rapier contact traps.** New geometry in a part's path, new pockets, new body-type changes:
   check against the measured list in CLAUDE.md (positive-distance contacts, stale contacts on
   kinematic links at rest, ≥ 12 mm pocket clearance, parts never sleep).
9. **Silent failure.** A skip must print why. A soak must check PROGRESS, not balance. A warning
   must not be silenced. An empty catch is a finding.
10. **Only then:** naming, comment density, dead code. Match the file's own idiom; this repo
    writes comments that say *why*, with the measurement in them.

## Reviewing a measurement

A benchmark that shared the CPU measured the CPU. If a diff carries timing numbers, check the run
had the box to itself; the same deterministic scene read 441 µs alone and 1232 µs alongside a test
suite. Sim-time results (`plant.run()`) are immune; wall-clock ones (`plant.start()`) are not.

## Output

Ranked most severe first, and nothing else:

```
<path>:<line>  <severity>  <one sentence: what is wrong>
   fails when: <concrete input or sequence that produces the wrong result>
   fix: <the smallest correct change>
```

Severity: **breaks** (wrong behaviour reaches the PLC, the picture or the disk), **silent** (works
now, fails without an error later), **test-gap** (behaviour not pinned), **style**.

Rules:
- No praise, no summary of what the code does, no "consider maybe".
- Every finding names the line and a failure that a person could reproduce.
- If a check found nothing, say "no findings" for it — do not invent work.
- Report a finding against your OWN earlier change with the same bluntness. The write-off at every
  START was caught that way.
