# uwb_gazebo_plugin

Simulated UWB (ultra-wideband) ranging for Gazebo Harmonic. Every link named
`uwb_anchor_<id>` measures its distance to every link named `uwb_tag_<id>`, with
noise, dropouts and biases that depend on distance and on what blocks the signal.

Vendored from [UWBPX4Sim](https://github.com/amartinezsilva/UWBPX4Sim) by
Andrés Martínez (MIT, see `LICENSE`), commit `b99d92e`. Only the plugin was
copied; that repo's PX4, Docker, GUI and drone code isn't needed here. Its docs
explain the model: `docs/uwb_basics.md` and `docs/measurement_model.md`.

## Using it

`./scripts/build.sh` builds it, and sourcing `install/setup.bash` (the scripts
do this) adds its `lib/` to `GZ_SIM_SYSTEM_PLUGIN_PATH`. A world turns it on
with a `<plugin filename="libUWBGazeboPlugin.so" name="custom::UWBGazeboSystem">`
block; print one from a preset with:

```bash
python3 src/uwb_gazebo_plugin/preset_to_sdf.py lps_twr
```

`simple_colored_warehouse.sdf` uses `lps_twr`: Bitcraze Loco Positioning (DWM1000 radios)
in two-way ranging mode, 30 readings per second per anchor-tag pair.

Topics (Gazebo, `gz.msgs.Double`, **centimetres**):

| Topic | |
|---|---|
| `/uwb_gz_simulator/distances/a<I>t<J>` | measured distance, anchor I to tag J |
| `/uwb_gz_simulator/distances_ground_truth/a<I>t<J>` | true distance |

```bash
gz topic -e -t /uwb_gz_simulator/distances/a8t1
```

## Rules for models

- Tags and anchors are links named `uwb_tag_<id>` / `uwb_anchor_<id>` directly
  on a model (not in a nested model). Attach them with a fixed joint.
- A UWB model's main link must be named `base_link` for its body to block the
  signal; its other links are ignored.
- Obstacles need `<collision>` geometry; visual-only objects don't block.
- The floor must be named exactly `ground_plane`.
- URDF robots (the TurtleBot): Gazebo merges fixed-joint links into their
  parent, so each UWB link needs `<preserveFixedJoint>true</preserveFixedJoint>`
  in a `<gazebo reference="...">` block, or it disappears.

## Measurements read short

With `lps_twr`, each anchor-tag pair gets a fixed offset of -10.5 to -34.4 cm,
like real DWM1000 radios before calibration. Line of sight, expect readings
about 10-35 cm below the true distance. The localizer will need to correct for
it (or set `apply_pair_bias` to `false` in the world while developing).

## Changes from upstream (marked `CS485` in the code)

- `CMakeLists.txt`: project renamed `uwb_gazebo_plugin` (the library is still
  `libUWBGazeboPlugin.so`); builds as an ament package with an environment hook
  for `GZ_SIM_SYSTEM_PLUGIN_PATH`, and installs the presets.
- `UWBGazeboSystem.cpp/.hpp`: a blocked link was logged on every update (30
  lines a second per pair); it's now logged only when a pair becomes blocked or
  clear again.
- Added `package.xml`, `env-hooks/`, this README, and `preset_to_sdf.py` (from
  upstream's `tools/`).
