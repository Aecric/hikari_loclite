# Bootstrap Task: Fill Project Development Guidelines

This task populates `.trellis/spec/` with real project rules for
`hikari_loclite`, a standalone ROS2 C++ fixed-map LiDAR localization package.
The main source document is `hikari_loclite_build_2026-06-10.md`.

---

## Status

- [x] Replace generic backend template with ROS2/C++ SLAM guidelines
- [x] Remove non-applicable frontend/database template specs
- [x] Add project-backed examples and references
- [x] Capture build, dependency, localization, runtime, error, logging, and quality constraints
- [x] Rewrite generic thinking guides for ROS2/SLAM data flow and reuse checks

---

## Spec files to populate


### Runtime C++ / ROS2 guidelines

| File | What to document |
|------|------------------|
| `.trellis/spec/backend/index.md` | Entry point and pre-development checklist |
| `.trellis/spec/backend/directory-structure.md` | ROS2 package layout, module ownership, forbidden full-framework includes |
| `.trellis/spec/backend/build-and-dependencies.md` | ament, C++17, PCL/Eigen/OpenMP, dependency allow/deny list |
| `.trellis/spec/backend/localization-architecture.md` | Fixed-map Fast-LIO, iVox, NDT, Scan Context, non-goals |
| `.trellis/spec/backend/runtime-and-relocalization.md` | Node contract, state machine, `/initialpose`, LOST recovery, pose gating |
| `.trellis/spec/backend/error-handling.md` | Return-value conventions and validation gates |
| `.trellis/spec/backend/logging-guidelines.md` | Runtime logging and hot-path log-rate rules |
| `.trellis/spec/backend/quality-guidelines.md` | Build command, forbidden patterns, review checklist |


### Thinking guides

`.trellis/spec/guides/` was adjusted so thinking triggers reference ROS2
callbacks, sensor buffers, Fast-LIO, NDT, Scan Context, TF, and config instead
of web/API/database layers.

---

## How to fill the spec

### Step 1: Import from existing convention files first (preferred)

Search the repo for existing convention docs. If any exist, read them and
extract the relevant rules into the matching `.trellis/spec/` files —
usually much faster than documenting from scratch.

| File / Directory | Tool |
|------|------|
| `CLAUDE.md` / `CLAUDE.local.md` | Claude Code |
| `AGENTS.md` | Codex / Claude Code / agent-compatible tools |
| `.cursorrules` | Cursor |
| `.cursor/rules/*.mdc` | Cursor (rules directory) |
| `.windsurfrules` | Windsurf |
| `.clinerules` | Cline |
| `.roomodes` | Roo Code |
| `.github/copilot-instructions.md` | GitHub Copilot |
| `.vscode/settings.json` → `github.copilot.chat.codeGeneration.instructions` | VS Code Copilot |
| `CONVENTIONS.md` / `.aider.conf.yml` | aider |
| `CONTRIBUTING.md` | General project conventions |
| `.editorconfig` | Editor formatting rules |

### Step 2: Analyze the codebase for anything not covered by existing docs

Scan real code to discover patterns. Before writing each spec file:
- Find 2-3 real examples of each pattern in the codebase.
- Reference real file paths (not hypothetical ones).
- Document anti-patterns the team clearly avoids.

### Step 3: Document reality, not ideals

**Critical**: write what the code *actually does*, not what it should do.
Sub-agents match the spec, so aspirational patterns that don't exist in the
codebase will cause sub-agents to write code that looks out of place.

If the team has known tech debt, document the current state — improvement
is a separate conversation, not a bootstrap concern.

---

## Quick explainer of the runtime (share when they ask "why do we need spec at all")

- Every AI coding task spawns two sub-agents: `trellis-implement` (writes
  code) and `trellis-check` (verifies quality).
- Each task has `implement.jsonl` / `check.jsonl` manifests listing which
  spec files to load.
- The platform hook auto-injects those spec files + the task's `prd.md`
  into every sub-agent prompt, so the sub-agent codes/reviews per team
  conventions without anyone pasting them manually.
- Source of truth: `.trellis/spec/`. That's why filling it well now pays
  off forever.

---

## Completion

Before archiving this task, verify:

```bash
python3 ./.trellis/scripts/get_context.py --mode packages
rg -n "To be filled|TODO: fill|placeholder|Replace with your actual" .trellis/spec
```

The packages command should report only the meaningful spec layers for this
repository, and the placeholder search should return no template placeholders.
