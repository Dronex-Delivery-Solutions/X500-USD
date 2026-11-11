# Holybro X500 for Isaac Sim

A USD asset of the Holybro X500 quadrotor, converted from the PX4 Gazebo model
(`PX4/PX4-gazebo-models`, `models/x500` + `models/x500_base`).

![preview](docs/preview.png)

**Status: built and checked with OpenUSD only. It has not been loaded in Isaac
Sim yet** — see [Before you rely on it](#before-you-rely-on-it).

## Use it

The file to load is `x500/x500.usda`. It needs `geometries.usdc` and
`textures/` next to it, so copy or reference the whole `x500/` folder.

```python
# plain Isaac Sim
from isaacsim.core.utils.stage import add_reference_to_stage
add_reference_to_stage("/abs/path/to/x500/x500.usda", "/World/x500")
```

For PX4 SITL through Pegasus Simulator see `examples/pegasus_px4_x500.py`.

Spawn it at z ≥ 0.24 m: the origin is the body frame, the skids are 0.227 m
below it.

## What is in the asset

| Prim | What |
|---|---|
| `/x500` | default prim, articulation root |
| `/x500/body` | airframe rigid body, 2.0 kg, inertia (0.0217, 0.0217, 0.04) kg·m² |
| `/x500/rotor0` … `rotor3` | rotor rigid bodies, 16 g each, at (±0.174, ±0.174, 0.06) m |
| `/x500/joints/joint0` … `joint3` | continuous revolute joints about Z |
| `…/visuals/*` | instanced references into `geometries.usdc` (207 584 triangles in total) |
| `…/collisions/*` | 5 boxes on the body, one disc per rotor |

- Units are metres and kilograms, Z up. Body frame is x forward, y left, z up.
- Rotor numbering is PX4 quad-X: 0 front-right, 1 rear-left (both
  counter-clockwise), 2 front-left, 3 rear-right (both clockwise).
- Prim names (`body`, `rotorN`, `jointN`) are the ones Pegasus Simulator looks up.
- Each rotor prim carries the Gazebo motor-model numbers as `x500:motor:*`
  attributes (`motorConstant` 8.54858e-06, `momentConstant` 0.016,
  `maxRotVelocity` 1000 rad/s, …). Nothing in Isaac Sim reads them
  automatically; they are there so your controller code can.

### Where it differs from the PX4 model

- Rotor colliders are discs (radius 0.14 m, 5 mm thick) instead of the SDF's
  0.279 × 0.017 × 0.0008 m boxes, which are too thin for PhysX to handle well.
- Roughness and metalness are hand-picked; the source materials only define a
  colour.
- The Gazebo sensors (IMU, barometer, magnetometer, GPS) and the motor plugin
  are not part of the USD. Add them in your simulation code.
- No Isaac robot-schema overlay (`IsaacRobotAPI`), because it cannot be
  authored without Isaac Sim. Tools that need it can add it with
  `usd.schema.isaac.robot_schema.ApplyRobotAPI` (robot type `Aerial`).

## Before you rely on it

Checked here, without Isaac Sim (`scripts/validate_x500.py`):

- one articulation root; every rotor connected to the body by a joint whose
  frames agree
- explicit mass and inertia on all five bodies, total 2.064 kg
- all references and textures resolve; every mesh has a material
- geometry placement, by eye, from `docs/preview.png`

Not checked — do this once in Isaac Sim:

1. Open `x500/x500.usda`, press Play on a ground plane: it should drop onto
   its skids and stay still.
2. Look at the materials under RTX.
3. Run the Pegasus example and take off.

## Rebuild

```bash
pip install -r requirements-build.txt
```

```bash
python scripts/build_x500.py
```

```bash
python scripts/validate_x500.py
```

```bash
python scripts/render_preview.py
```

`build_x500.py` downloads the PX4 meshes into `.cache/` and regenerates
`x500/`. Change numbers in the script, not in the generated `.usda`.

## Add it to a project

```bash
git submodule add https://github.com/Dronex-Delivery-Solutions/X500-USD.git assets/X500-USD
```

Then load `assets/X500-USD/x500/x500.usda`.

## License

BSD-3-Clause, see `LICENSE`. The meshes, textures and physical parameters come
from [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models), Copyright
(c) 2022 PX4 Autopilot for Drones; the original notice is kept verbatim in
`LICENSE-PX4-gazebo-models`. Keep both files with the asset wherever you copy
or publish it.
