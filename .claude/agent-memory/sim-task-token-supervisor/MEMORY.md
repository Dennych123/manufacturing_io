# Agent memory — sim-task-token-supervisor (manufacturing_io)

- [File responsibility map](project_file_map.md) — which module owns scene JSON, worldPoses, driver, OPC UA, sysmac gen; targeted-read anchors.
- [Token-heavy files](feedback_token_heavy.md) — what to grep vs read wholesale; docs/PLAN.md is 622 lines, load once.
- [Phase state](project_phase_state.md) — phase boundaries and resume point at last session end.
- [User profile](user_profile.md) — Denny: PLC/automation engineer, rb4axis/ceinsert/sysmac author.
- [Working cadence](feedback_cadence.md) — compact ~every 5h before usage limit; resume after reset.
- [Invariant scan recipe](feedback_invariant_scan.md) — the cheap grep that checks the silent-failure rules.
