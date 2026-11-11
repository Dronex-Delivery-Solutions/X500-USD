#!/usr/bin/env python3
"""Structural checks for a sensor-accessory USD (sensors/<name>/<name>.usda),
needing only usd-core. Generalizes scripts/validate_x500.py's checks to a
stage that REFERENCES x500/x500.usda and adds one or more rigid bodies.

    python scripts/validate_sensor_variant.py sensors/lidar_down/lidar_down.usda

Does not (cannot) validate OmniSensorGenericLidarCoreAPI or RTX camera
behaviour - those schemas are Isaac Sim-only. This checks what plain USD
can see: composition, physics bodies/joints, and the attributes this
repo's own build scripts are supposed to have authored.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

BASE_BODIES = {"body", "rotor0", "rotor1", "rotor2", "rotor3"}
LIDAR_SCHEMA = "OmniSensorGenericLidarCoreAPI"


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    path = Path(sys.argv[1])
    stage = Usd.Stage.Open(str(path))
    errors: list[str] = []

    def check(ok: bool, message: str):
        print(("  ok   " if ok else "  FAIL ") + message)
        if not ok:
            errors.append(message)

    print(f"{path}")
    root = stage.GetDefaultPrim()
    check(bool(root), "defaultPrim is set")
    check(root.HasAPI(UsdPhysics.ArticulationRootAPI), "ArticulationRootAPI present (inherited from x500.usda)")

    everything = list(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
    bodies = {p.GetName(): p for p in everything if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    check(BASE_BODIES <= bodies.keys(), f"base vehicle bodies present: {sorted(bodies.keys() & BASE_BODIES)}")
    accessory_bodies = {n: p for n, p in bodies.items() if n not in BASE_BODIES}
    cameras_added = [p for p in everything if p.IsA(UsdGeom.Camera)]
    lidars_added = [p for p in everything if LIDAR_SCHEMA in str(p.GetMetadata("apiSchemas") or "")]
    check(
        len(accessory_bodies) > 0 or cameras_added or lidars_added,
        f"accessory present: {list(accessory_bodies) or [c.GetName() for c in cameras_added] or [l.GetName() for l in lidars_added]}"
        + (" (no new rigid body - lightweight accessory folded into an existing body, by the build script's own choice)"
           if not accessory_bodies else ""),
    )

    for name, body in accessory_bodies.items():
        mass_api = UsdPhysics.MassAPI(body)
        mass = mass_api.GetMassAttr().Get() or 0.0
        check(body.HasAPI(UsdPhysics.MassAPI) and mass > 0, f"{name}: explicit positive mass ({mass} kg)")
        colliders = [c for c in Usd.PrimRange(body) if c.HasAPI(UsdPhysics.CollisionAPI)]
        check(len(colliders) > 0, f"{name}: has a collider")

    # Every rigid body must be reachable from "body" through some chain of
    # Revolute or Fixed joints (accepts both, unlike the base-only validator).
    joints = [p for p in everything if p.IsA(UsdPhysics.Joint)]
    edges = {}
    cache = UsdGeom.XformCache()
    for joint in joints:
        j = UsdPhysics.Joint(joint)
        b0, b1 = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
        ok = len(b0) == 1 and len(b1) == 1 and b0[0] in [bp.GetPath() for bp in bodies.values()] and \
            b1[0] in [bp.GetPath() for bp in bodies.values()]
        check(ok, f"{joint.GetName()} ({joint.GetTypeName()}): body0/body1 resolve to rigid bodies")
        if ok:
            edges[b1[0]] = b0[0]
            w0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(b0[0])).Transform(j.GetLocalPos0Attr().Get())
            w1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(b1[0])).Transform(j.GetLocalPos1Attr().Get())
            check((w0 - w1).GetLength() < 1e-5, f"{joint.GetName()}: joint frames agree ({(w0 - w1).GetLength():.2g} m)")
    body_paths = {p.GetPath(): n for n, p in bodies.items()}
    reached = {root.GetPath().AppendChild("body")}
    frontier = True
    while frontier:
        frontier = False
        for child, parent in edges.items():
            if parent in reached and child not in reached:
                reached.add(child)
                frontier = True
    check(reached == set(body_paths), f"every body reachable from 'body' ({len(reached)}/{len(body_paths)})")

    # Sensor-schema prims this repo's build scripts author (best-effort: the
    # schema itself is Isaac Sim-only, so this checks our own attribute
    # authoring, not Isaac Sim's interpretation of it).
    for lidar in lidars_added:
        az = lidar.GetAttribute("omni:sensor:Core:emitterState:s001:azimuthDeg").Get()
        el = lidar.GetAttribute("omni:sensor:Core:emitterState:s001:elevationDeg").Get()
        ft = lidar.GetAttribute("omni:sensor:Core:emitterState:s001:fireTimeNs").Get()
        mn = lidar.GetAttribute("minRange").Get()
        mx = lidar.GetAttribute("maxRange").Get()
        check(az is not None and len(az) == len(el) == len(ft) and len(az) > 0,
              f"{lidar.GetName()}: emitter arrays present and equal length ({len(az) if az else 0} beams)")
        check(mn is not None and mx is not None and 0 < mn < mx, f"{lidar.GetName()}: 0 < minRange({mn}) < maxRange({mx})")
        if ft and len(ft) > 1:
            check(all(b >= a for a, b in zip(ft, ft[1:])), f"{lidar.GetName()}: fireTimeNs non-decreasing")

    for cam in cameras_added:
        c = UsdGeom.Camera(cam)
        focal = c.GetFocalLengthAttr().Get()
        h_ap = c.GetHorizontalApertureAttr().Get()
        near, far = c.GetClippingRangeAttr().Get()
        check(focal and focal > 0 and h_ap and h_ap > 0, f"{cam.GetName()}: positive focal length ({focal}) and aperture ({h_ap})")
        check(0 < near < far, f"{cam.GetName()}: 0 < near({near}) < far({far})")

    meshes = [p for p in everything if p.IsA(UsdGeom.Mesh)]
    unbound = [str(m.GetPath()) for m in meshes if not UsdShade.MaterialBindingAPI(m).ComputeBoundMaterial()[0]]
    check(not unbound, f"every mesh has a material {unbound[:3] if unbound else ''}")
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
    check(not unresolved, f"all references/textures resolve ({len(layers)} layers, {len(assets)} assets) {unresolved or ''}")

    print("\nFAILED" if errors else "\nall checks passed (schema-specific attributes still need an Isaac Sim load to confirm)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
