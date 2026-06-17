# Update README acknowledgements and MIT license

## Goal

Document the open-source projects and reference implementations used by this repository, and set the repository license to MIT.

## What I already know

- User requested a README update after recent changes.
- README should include a line for GitHub open-source libraries such as KISS-Matcher and references such as lightning-lm.
- The repository currently has no top-level `LICENSE` file.
- `package.xml` currently declares `<license>Proprietary</license>`.

## Requirements

- Add README text acknowledging GitHub open-source libraries and reference projects.
- Add a top-level MIT `LICENSE` file.
- Update ROS package metadata to declare the MIT license.
- Keep the change scoped to documentation and package metadata.

## Acceptance Criteria

- [x] README includes the acknowledgement in Chinese and English sections.
- [x] Top-level `LICENSE` contains the MIT license text.
- [x] `package.xml` license is `MIT`.
- [x] No unrelated working-tree changes are modified.

## Out of Scope

- No runtime code behavior changes.
- No dependency or build-system restructuring.
- No third-party license rewriting.

## Technical Notes

- Existing third-party code under `thirdparty/3rd/KISS-Matcher/` retains its own license files.
- `README.md` is bilingual, so additions should remain bilingual.
