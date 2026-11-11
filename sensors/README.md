# X500 sensor accessories

Three additive USD layers, each adding one sensor onto the X500 in
`../x500/x500.usda` without modifying it. All three converted from PX4's own
Gazebo sensor variants (`PX4/PX4-gazebo-models`, BSD-3-Clause), same as the
base vehicle.

| Accessory | PX4 source | Adds | Mass | Mount |
|---|---|---|---|---|
| `lidar_down/` | `x500_lidar_down` (LW20) | 1-beam rangefinder, straight down | 1 g | z = -0.05 m |
| `lidar_2d_front/` | `x500_lidar_2d` (lidar_2d_v2) | 270°, 1080-beam planar scan, forward | 370 g | on a mast at z = +0.315 m |
| `camera_depth/` | `x500_depth` (OAK-D Lite) | RGB (1920×1080) + depth (640×480) camera, forward | 61 g, folded into `/x500/body` | z = +0.242 m |

`full_stack.usda` combines all three on one vehicle (verified: USD composes
the repeated references to `x500.usda` without duplicating the airframe —
see the commit that added it). Pick a subset of its `subLayers` instead of
using it as-is if a vehicle only needs some of them.

Use any of these exactly like `x500/x500.usda` — reference `<name>.usda`,
keep its folder (mesh + textures) alongside it. Rebuild with
`python sensors/<name>/build.py`; validate with
`python scripts/validate_sensor_variant.py sensors/<name>/<name>.usda`.
Both the mast height on the 2D lidar and the camera's height are PX4's own
numbers, reproduced exactly — not something tuned for this project.

**Status: loaded and checked against a real Isaac Sim 6.0.0** (headless,
2026-11-12) — see **What's authored vs. what's confirmed** below for
exactly what that covered and what it didn't. Two real authoring mistakes
turned up and were fixed that way, not by guessing twice; the lidar and
camera prims here have been run through `Lidar()`/`RtxCamera()`, not just
built and hoped for.

## The feasibility question: can PX4-in-sim actually use these?

Short answer: the lidar and camera **assets** are fine to build and will
very likely load in Isaac Sim 6 — the geometry/mass/joint layer uses the
same USD Physics schemas as the base vehicle, which nothing has changed
underneath. The gap is in **getting the data out of Isaac Sim 6 and into
PX4 or ROS 2**, and it is real:

**Pegasus Simulator's latest release targets Isaac Sim 5.1.0** (its
`extension.toml` still depends on the pre-6.0 `omni.isaac.core` package;
there is no 6.0 entry in its changelog, on `main` or on its `dev` branch,
as of when this was written). Two consequences, read directly out of
Pegasus's own source (`pegasus/simulator/logic/graphical_sensors/`):

- **Camera**: `MonocularCamera` imports
  `isaacsim.sensors.camera.camera.Camera`. In Isaac Sim 6 that extension is
  described in Isaac Sim's own docs as "a stub (no Python)" — the
  implementation moved to `isaacsim.sensors.experimental.rtx`. Pegasus's
  built-in camera sensor is therefore at real risk of not producing images
  on Isaac Sim 6, independent of anything in this repo.
- **Lidar**: Pegasus's `Lidar` class is commented out of
  `graphical_sensors/__init__.py` on *both* `main` and `dev` — it isn't
  even importable today, on any Isaac Sim version. Reading the dead code,
  it also calls `IsaacSensorCreateRtxLidar`, the pre-6.0 lidar-authoring
  kit command.
- **What isn't affected**: the vehicle's core flight loop
  (IMU/GPS/baro/mag → MAVLink, rotor forces) goes through
  `omni.isaac.dynamic_control`, which Isaac Sim's own migration notes list
  as a legacy API that still loads in 6.0 with a deprecation warning, not
  an error. So the base vehicle flying under Pegasus is lower-risk than
  any of its sensor classes — but, per the main README, still unconfirmed.

None of this is a reason to not build the assets — it changes **how you
get data out of them**, not whether they're worth having. The practical
split:

### Downward rangefinder → PX4 (needed for sim testing per your plan)

PX4 consumes this as a MAVLink `DISTANCE_SENSOR` message
(`SimulatorMavlink::handle_message_distance_sensor` in PX4-Autopilot),
which feeds `EKF2_RNG_*`/terrain estimation. Pegasus's PX4 backend does
not send this message (its `update_sensor()` only forwards
IMU/GPS/Barometer/Magnetometer). `examples/distance_sensor_backend.py`
is a first attempt at closing that gap — a `PX4MavlinkBackend` subclass
that raycasts straight down from the `lidar_down` prim (a stable, basic
PhysX query, not the RTX lidar pipeline) and sends the MAVLink message
itself. **Not tested** — written from reading PX4's message handler and
Pegasus's own `send_sensor_msgs`/`send_gps_msgs` pattern, not from running
it.

### 2D lidar and camera → companion computer, even in sim

Neither has a MAVLink message PX4's simulator interface accepts (a
1080-beam scan and an image have no SITL counterpart), and that's fine:
your own plan already puts these on a companion computer at deployment, so
wiring them the same way in sim — RTX lidar / RTX camera → ROS 2
(`ROS2PublishLaserScan`, `ROS2PublishPointCloud`, standard image
publishers) — tests the real architecture instead of a simulation-only
shortcut. This sidesteps Pegasus's graphical-sensor classes entirely by
using Isaac Sim 6's own `isaacsim.sensors.experimental.rtx` APIs directly,
which is what Isaac Sim 6 itself documents as current, and both prims are
now confirmed to load through it (`Lidar()`, `RtxCamera()` — see **What's
authored vs. what's confirmed**). The ROS 2 publishing side itself (the
writer/`ROS2PublishPointCloud` wiring) is not implemented here — no ROS 2
to test against on the machine this was built on.

### If you'd rather avoid the RTX pipeline entirely for the 2D lidar

A straight-down raycast is simple because it's one ray. A 1080-beam scan
is not something to re-implement as 1080 raycasts per frame without a
reason to, and the RTX lidar path is now confirmed to load, so there's
less reason to avoid it than there was. If you still want to: the
fallback is a PhysX `raycast_closest` sweep across the same 1080 angles
(same primitive `examples/distance_sensor_backend.py` uses), which is
slower and less physically accurate (no multi-return, no material-based
intensity) but has no dependency on RTX sensor APIs at all. Worth knowing
about, not written here.

## Other sensor options, for later

Not built here, in rough order of how much they'd add for a delivery UAV:

- **3D lidar** (Ouster, Hesai, Velodyne, …): Isaac Sim 6 ships these as
  ready-made vendor configs (`isaacsim.sensors.experimental.rtx.
  SUPPORTED_LIDAR_CONFIGS`) — a mount point and one `Lidar.create(config=,
  variant=)` call, no custom scan-pattern authoring needed. The most
  useful upgrade from the 2D lidar here if real obstacle maps/SLAM matter.
- **Stereo depth** (RealSense D435/D455, ZED 2i): also ready-made Isaac
  Sim configs; a direct swap for the OAK-D if you'd rather match hardware
  you actually have on the bench.
- **Radar / ultrasonic**: both exist as RTX sensors in Isaac Sim 6;
  radar in particular matters for low-visibility delivery flights.
  Not pursued here for lack of a clear use case yet.
- **Payload/winch**: not a sensor, but the next thing a delivery airframe
  usually needs — a mass + joint problem like the sensors here, not a new
  category of difficulty.

## What's authored vs. what's confirmed

**Confirmed against a live Isaac Sim 6.0.0** (headless, no GUI, run via a
Windows install through WSL interop — see the commit that updated this
section for the exact scripts):

1. The base vehicle, all three accessories and `full_stack.usda` all open
   cleanly with `ArticulationRootAPI` intact.
2. The base vehicle, referenced into a fresh stage the documented way
   (`add_reference_to_stage`) with a ground plane, falls under gravity and
   settles at **z = 0.2270 m** starting from z = 1.0 m — matching the
   documented 0.227 m skid offset almost to the decimal, over 60 physics
   steps with nothing diverging.
3. `isaacsim.sensors.experimental.rtx.Lidar("/x500/.../sensor")` wraps both
   the 1-beam and the 1080-beam lidar prims without error. **This needed a
   fix**: the first version authored a plain `Xform` with
   `OmniSensorGenericLidarCoreAPI` applied, following the isaac-sim-sensor
   skill's documentation literally — `Lidar()` actually requires the
   prim's USD type itself to be `OmniLidar`, and several attribute names
   the skill's example showed (`minRange`, `maxRange`, `horizontalFov`,
   `horizontalResolution`, `verticalFov`, `verticalResolution`, `highLod`,
   `drawPoints`) don't exist in the real schema at all — confirmed absent
   from a ~70-attribute dump of a reference lidar `Lidar.create()` built
   itself. The real range attributes are
   `omni:sensor:Core:nearRangeM`/`farRangeM`. The emitter-state attribute
   names and the applied-schema token were already right the first time.
4. `isaacsim.sensors.experimental.rtx.RtxCamera("/x500/body/cameras/rgb")`
   wraps the camera prim without error, and a `CameraSensor` built on top
   of it returns non-`None` frame data after a few sim ticks. **Also
   needed a fix**: `RtxCamera()` requires `OmniSensorAPI` applied to the
   camera prim, which the first version didn't have. (Returning
   non-`None` data is a weaker signal than the other three checks here —
   it wasn't inspected for being a genuine non-empty image versus an
   early placeholder buffer.)
5. The camera prims' optical axis is correct: read back from a live
   stage, `look_dir=(1,0,0)` and `up_dir=(0,0,1)` — exactly body-forward
   and level, confirming `_usd_common.FORWARD_CAMERA_ROTATION` (derived by
   hand, previously verified only in a unit test, not against Isaac Sim).

**Still not confirmed:**

1. The vehicle flying under **Pegasus Simulator** specifically (PX4
   MAVLink, rotor thrust from motor commands) — everything above used
   Isaac Sim's plain physics/sensor APIs directly, not Pegasus. The
   Pegasus-specific compatibility concerns below are still reasoned from
   reading Pegasus's source, not from running it.
2. `examples/distance_sensor_backend.py` — no PX4 SITL instance was run
   against this.
3. The 2D lidar's actual point cloud / the camera's actual image content
   looking correct, as opposed to the wrapper classes accepting the prims
   without error.

## License

Same as the base vehicle: BSD-3-Clause, PX4 Autopilot for Drones — see
`../LICENSE` and `../LICENSE-PX4-gazebo-models`.
