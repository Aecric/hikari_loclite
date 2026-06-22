# Separate Shared Reloc Config From SC Config

## Goal

Remove ambiguity in `reloc` configuration by separating backend-agnostic KISS/SC scheduling options from SC-only fallback options. KISS is the default relocalization backend, so shared knobs must not live under comments that imply they only affect Scan Context.

## Requirements

- Move/comment YAML options so KISS/SC shared scheduling and query-preparation knobs are grouped separately from SC-only fallback knobs.
- Rename shared runtime keys away from `sc_*` where they are used by KISS too, while keeping legacy `sc_*` fallback reads for existing configs.
- Update C++ accessors/call sites so shared semantics are named `Reloc*` / query accumulation, not `Sc*`, where practical.
- Keep SC-specific options and SC code available for `reloc_backend=sc`.
- Do not change current runtime defaults or backend behavior.

## Acceptance Criteria

- [ ] `config/loclite_livox.yaml` clearly distinguishes shared reloc scheduling/query accumulation from KISS-only and SC-only settings.
- [ ] New shared keys are read first, with old SC-prefixed keys as compatibility fallbacks.
- [ ] KISS dispatch uses shared cooldown naming in code.
- [ ] SC-only comments no longer describe KISS-shared options as SC behavior.
- [ ] Static checks pass.

## Definition of Done

- C++ compiles or a build-blocker is reported.
- `git diff --check` passes.
- No unrelated refactor or dependency changes.

## Technical Approach

- Add shared YAML keys such as `reloc_cooldown_sec`, `query_accum_frames`, `query_accum_voxel_leaf`, and `query_accum_max_rel_trans_m`.
- Keep legacy compatibility: `sc_cooldown_sec`, `sc_accum_frames`, `sc_accum_voxel_leaf`, and `sc_accum_max_rel_trans_m` remain fallback reads.
- Rename internal cooldown/accumulation members and getters to shared names, while preserving compatibility getters if needed.

## Out of Scope

- Removing Scan Context implementation.
- Renaming the existing `hikari_loc/sc_reloc` service.
- Changing default values, relocalization cadence, or LOST/WAIT state behavior.

## Technical Notes

- Relevant config: `config/loclite_livox.yaml`.
- Relevant parser: `src/reloc/reloc_manager.cpp`.
- Relevant runtime dispatch: `src/system/loclite_node.cpp`.
- Specs read: `.trellis/spec/backend/runtime-and-relocalization.md`, `.trellis/spec/backend/localization-architecture.md`, shared code-reuse and cross-layer guides.
