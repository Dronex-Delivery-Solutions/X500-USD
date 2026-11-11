#!/usr/bin/env python
"""Feed the downward LW20 rangefinder (sensors/lidar_down) to PX4 as a
MAVLink DISTANCE_SENSOR message, for EKF2 terrain estimation and precision
landing (EKF2_RNG_CTRL, EKF2_RNG_AID and friends).

NOT TESTED - written from reading PX4's SimulatorMavlink::handle_message_
distance_sensor (src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp)
and Pegasus Simulator's PX4MavlinkBackend.send_sensor_msgs/send_gps_msgs,
not from running it. Needs Isaac Sim + Pegasus + PX4 SITL to confirm.

Why this exists at all: Pegasus's PX4MavlinkBackend.update_sensor() only
forwards IMU/GPS/Barometer/Magnetometer; nothing in Pegasus sends
DISTANCE_SENSOR, and Pegasus's own Lidar graphical-sensor class is
commented out of its package (both on `main` and `dev`) and targets a
pre-6.0 Isaac Sim API either way - see sensors/README.md for the full
compatibility picture. This fills that one gap with the smallest thing
that can work: a straight-down PhysX raycast (a stable, long-standing
Isaac Sim API, not part of the Isaac Sim 6 RTX-sensor churn that affects
Pegasus's camera/lidar classes), not the OmniSensorGenericLidarCoreAPI
prim this repo's USD carries. That prim is there for later - a full RTX
lidar pipeline, or visualizing the beam - it is not what this script reads.

Usage: subclass your flight script's PX4MavlinkBackend with
DownwardRangefinderBackend instead, giving it the lidar's prim path.
"""
from __future__ import annotations

import time

import carb
import numpy as np
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pymavlink import mavutil

# MAV_DISTANCE_SENSOR: LASER (the LW20 is a laser rangefinder, not sonar/IR/radar).
MAV_DISTANCE_SENSOR_LASER = 1
# MAV_SENSOR_ORIENTATION: PITCH_270, the standard PX4/ArduPilot convention for a
# sensor that points straight down (matches the x500:motor:* custom-attribute
# pattern in x500/x500.usda: metadata travels with the asset, but orientation
# is a MAVLink enum, not something to read back out of the USD transform here).
MAV_SENSOR_ROTATION_PITCH_270 = 25

# From sensors/lidar_down/build.py (MIN_RANGE, MAX_RANGE), in metres.
MIN_RANGE_M, MAX_RANGE_M = 0.1, 100.0
UPDATE_RATE_HZ = 50.0  # matches the LW20's own update_rate in the PX4 SDF


class DownwardRangefinderBackend(PX4MavlinkBackend):
    """PX4MavlinkBackend + one straight-down raycast sent as DISTANCE_SENSOR.

    Args:
        lidar_prim_path: USD path of the lidar_down rigid body, e.g.
            "/World/quadrotor/lidar_down" (stage_prefix + "/lidar_down").
        config: as PX4MavlinkBackendConfig.
    """

    def __init__(self, lidar_prim_path: str, config: PX4MavlinkBackendConfig = PX4MavlinkBackendConfig()):
        super().__init__(config)
        self._lidar_prim_path = lidar_prim_path
        self._last_range_sent_time = 0.0
        self._sensor_id = 0

    def _raycast_down(self) -> float | None:
        """Straight-down distance from the lidar_down prim to whatever it
        hits, in metres, or None if nothing is within MAX_RANGE_M.
        """
        import omni.physx
        from pxr import UsdGeom
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._lidar_prim_path)
        if not prim.IsValid():
            carb.log_warn(f"DownwardRangefinderBackend: no prim at {self._lidar_prim_path}")
            return None

        origin = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
        query = omni.physx.get_physx_scene_query_interface()
        hit = query.raycast_closest(
            carb.Float3(origin[0], origin[1], origin[2]),
            carb.Float3(0.0, 0.0, -1.0),  # world -Z: the lidar_down joint is
            # mounted so its own local +X beam already points world -Z when
            # the vehicle is level - this raycast uses world -Z directly
            # instead, which is only exactly "the beam direction" when level.
            # Re-deriving the true beam direction from the prim's own
            # transform (its local +X axis) is the more correct fix if this
            # needs to work while banked.
            MAX_RANGE_M,
        )
        if not hit["hit"]:
            return None
        return float(hit["distance"])

    def update(self, dt: float):
        super().update(dt)

        if not self._received_first_hearbeat:
            return
        if (time.time() - self._last_range_sent_time) < (1.0 / UPDATE_RATE_HZ):
            return
        self._last_range_sent_time = time.time()

        distance_m = self._raycast_down()
        if distance_m is None or not (MIN_RANGE_M <= distance_m <= MAX_RANGE_M):
            return

        # DISTANCE_SENSOR carries distances in centimetres (uint16).
        self._connection.mav.distance_sensor_send(
            int(self._current_utime / 1000),  # time_boot_ms
            int(MIN_RANGE_M * 100),
            int(MAX_RANGE_M * 100),
            int(np.clip(distance_m * 100, MIN_RANGE_M * 100, MAX_RANGE_M * 100)),
            MAV_DISTANCE_SENSOR_LASER,
            self._sensor_id,
            MAV_SENSOR_ROTATION_PITCH_270,
            0,  # covariance: unknown
        )
