# Migrate Tuned LocLite Livox Config To New Reloc Semantics

## Goal

Create a migrated copy of `/home/aecriclin/下载/loclite_livox (1).yaml` using the latest config semantics and comments, preserving the user's tuned parameter values.

## Requirements

- Output file: `/home/aecriclin/下载/loclite_livox_new.yaml`.
- Use the current repo `config/loclite_livox.yaml` as the semantic/comment template.
- Preserve tuned scalar/list values from the source config where the same setting exists.
- Migrate old relocalization keys to the new shared names:
  - `reloc.sc_cooldown_sec` -> `reloc.reloc_cooldown_sec`
  - `reloc.sc_accum_frames` -> `reloc.query_accum_frames`
  - `reloc.sc_accum_voxel_leaf` -> `reloc.query_accum_voxel_leaf`
  - `reloc.sc_accum_max_rel_trans_m` -> `reloc.query_accum_max_rel_trans_m`
  - `reloc.sc_max_delta_trans_m` -> `reloc.reloc_max_delta_trans_m`
  - `reloc.sc_max_delta_rot_deg` -> `reloc.reloc_max_delta_rot_deg`
- Keep new template keys that the old file does not have.

## Acceptance Criteria

- [ ] Output YAML exists at `/home/aecriclin/下载/loclite_livox_new.yaml`.
- [ ] Output parses as YAML.
- [ ] Output contains new keys and does not contain old `sc_cooldown_sec`, `sc_accum_*`, or `sc_max_delta_*` active keys.
- [ ] Tuned values from the old file are preserved in their new locations.

## Out of Scope

- Changing runtime code.
- Tuning parameters beyond preserving the provided file's values.
- Committing the generated external config file.
