# Thinking Guides

Use these guides to catch integration mistakes before changing the ROS2
localization stack.

## Available Guides

| Guide | Purpose | When to Use |
|-------|---------|-------------|
| [Code Reuse Thinking Guide](./code-reuse-thinking-guide.md) | Identify repeated algorithm/config patterns | When adding utilities, constants, filters, or conversion helpers |
| [Cross-Layer Thinking Guide](./cross-layer-thinking-guide.md) | Think through data flow across modules | When a change crosses ROS2 callbacks, sensor buffers, Fast-LIO, NDT, SC, TF, or config |

## SLAM-Specific Thinking Triggers

Read [Cross-Layer Thinking Guide](./cross-layer-thinking-guide.md) when:

- A change touches sensor message conversion plus algorithm state.
- A change affects frame IDs, timestamps, deskewing, TF, or odometry output.
- A candidate pose can flow from `/initialpose`, Scan Context, NDT, and
  `ResetToMapPose()`.
- A config value affects CPU budget, map size, or relocalization frequency.

Read [Code Reuse Thinking Guide](./code-reuse-thinking-guide.md) when:

- Adding a new point type, conversion helper, or YAML option.
- Adding another nearest-neighbor, voxel, or filtering path.
- Copying code from `lightning-lm`.
- Modifying a constant used by ESKF, iVox, NDT, Scan Context, or preprocessing.

## Pre-Modification Rule

Before changing any config key, constant, topic name, frame ID, or threshold,
search for existing references:

```bash
rg "value_or_key_to_change" .
```

This prevents mismatches across CMake, YAML, launch files, callbacks, algorithm
defaults, and documentation.
