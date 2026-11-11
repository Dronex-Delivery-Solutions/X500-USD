#!/usr/bin/env python
"""Fly the X500 with PX4 SITL through Pegasus Simulator.

Adapted from Pegasus' examples/1_px4_single_vehicle.py; run it the same way:

    ISAACSIM_PYTHON examples/pegasus_px4_x500.py

NOT TESTED by the asset's build pipeline (it needs Isaac Sim + Pegasus + PX4).
Import paths follow Pegasus `main`; adjust them to your Isaac Sim version if
needed (e.g. `omni.isaac.core` vs `isaacsim.core.api`).
"""
from pathlib import Path

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from scipy.spatial.transform import Rotation

X500_USD = str(Path(__file__).resolve().parent.parent / "x500" / "x500.usda")

# Values from PX4-gazebo-models/models/x500/model.sdf (also stored on the
# rotor prims as x500:motor:* attributes).
MOTOR_CONSTANT = 8.54858e-06  # thrust [N] = MOTOR_CONSTANT * omega^2
MOMENT_CONSTANT = 0.016       # yaw torque [Nm] = MOMENT_CONSTANT * thrust
MAX_ROT_VELOCITY = 1000.0     # rad/s


def x500_config() -> MultirotorConfig:
    config = MultirotorConfig()
    config.thrust_curve = QuadraticThrustCurve(
        {
            "num_rotors": 4,
            "rotor_constant": [MOTOR_CONSTANT] * 4,
            # Pegasus computes yaw torque as coefficient * omega^2
            "rolling_moment_coefficient": [MOMENT_CONSTANT * MOTOR_CONSTANT] * 4,
            # Pegasus convention: -1 = counter-clockwise, 1 = clockwise
            "rot_dir": [-1, -1, 1, 1],
            "min_rotor_velocity": [0, 0, 0, 0],
            "max_rotor_velocity": [MAX_ROT_VELOCITY] * 4,
        }
    )
    # The PX4 model has no body drag term; this is Pegasus' default, tune to taste.
    config.drag = LinearDrag([0.50, 0.30, 0.0])
    return config


class PegasusApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Box Room"])

        config = x500_config()
        config.backends = [
            PX4MavlinkBackend(
                PX4MavlinkBackendConfig(
                    {
                        "vehicle_id": 0,
                        "px4_autolaunch": True,
                        "px4_dir": self.pg.px4_path,
                        "px4_vehicle_model": "gz_x500",  # PX4 airframe 4001
                    }
                )
            )
        ]

        Multirotor(
            "/World/quadrotor",
            X500_USD,
            0,
            [0.0, 0.0, 0.24],  # the skids sit 0.227 m below the body origin
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )
        self.world.reset()

    def run(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    PegasusApp().run()
