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

**Status, same as the base vehicle: built and checked with OpenUSD only,
not loaded in Isaac Sim.** The physics-composition checks
(`validate_sensor_variant.py`) are solid; the sensor-specific attributes
(`OmniSensorGenericLidarCoreAPI`, the camera's focal length under RTX) can
only be confirmed by Isaac Sim itself — see **What's authored vs. what's
confirmed** below.

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
which is what Isaac Sim 6 itself documents as current. Not implemented
here yet (no ROS 2 to test against in this sandbox); the USD side (camera
prim, lidar prim with the real PX4 beam pattern) is ready for it.

### If you'd rather avoid the RTX/Pegasus uncertainty entirely for the 2D lidar

A straight-down raycast is simple because it's one ray. A 1080-beam scan
is not something to re-implement as 1080 raycasts per frame without a
reason to. If the RTX lidar path turns out not to work on your Isaac Sim 6
install, the fallback is a PhysX `raycast_closest` sweep across the same
1080 angles (same primitive `examples/distance_sensor_backend.py` uses),
which is slower and less physically accurate (no multi-return, no
material-based intensity) but has no dependency on RTX sensor APIs at all.
Worth knowing about, not written here — try the RTX path first.

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

Same split as the main README, extended to the sensor-specific pieces:

**Checked here** (`scripts/validate_sensor_variant.py`, no Isaac Sim):
physics composition (one articulation root, every accessory body massed,
collided and jointed to the airframe), references/textures resolve, and —
this repo's own authoring only, not Isaac Sim's interpretation of it — the
lidar emitter arrays and camera focal length/aperture are present, equal
length, and in range.

**Needs an Isaac Sim 6 load to confirm:**

1. The vehicle still flies under Pegasus (per the main README — unchanged
   by adding sensors).
2. `OmniSensorGenericLidarCoreAPI` is a proprietary Kit schema, not part of
   open-source USD — it isn't registered in the `usd-core` this was built
   with, so the schema name and attribute names below were authored
   exactly as Isaac Sim's own agent-skill documentation states them, but
   never round-tripped through the real schema plugin. If it doesn't load
   cleanly, that's the first thing to check against Isaac Sim 6's current
   docs.
3. The camera prims' optical axis: derived from the fixed rotation between
   USD/OpenGL camera convention (looks down -Z) and the ROS/Gazebo optical
   convention PX4's SDF camera sensors use (looks down local +X, matching
   the x500 body's forward axis when unrotated) — worked out and
   numerically verified in this repo (`_usd_common.FORWARD_CAMERA_ROTATION`
   and `matrix_to_quat`'s own test battery), not confirmed against a
   rendered Isaac Sim frame.
4. `examples/distance_sensor_backend.py`, as stated above.

## License

Same as the base vehicle: BSD-3-Clause, PX4 Autopilot for Drones — see
`../LICENSE` and `../LICENSE-PX4-gazebo-models`.
