---
name: feedback-cadence
description: Working cadence — compact ~every 5h before the usage limit, resume after reset.
metadata:
  type: feedback
---

Compact/summarize the session roughly every 5 hours, before hitting the usage limit; resume after the limit window resets.

**Why:** user works in long sessions and wants to avoid losing context to an abrupt limit; a checkpoint at a natural boundary lets earlier verbose context be dropped cleanly.
**How to apply:** emit a checkpoint summary at phase boundaries or when context grows large, and proactively recommend compaction near the ~5h mark. On resume, read [[project-phase-state]] first.
