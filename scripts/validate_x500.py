#!/usr/bin/env python3
"""Structural checks for x500/x500.usda that need only usd-core (no Isaac Sim).

    python scripts/validate_x500.py [path/to/x500.usda]

Exits non-zero on the first category of failure. This does NOT prove the
asset simulates correctly - only Isaac Sim can - but it catches the mistakes
that make an articulation silently fall apart: several articulation roots,
joints pointing at missing bodies, unresolved references, missing masses.
"""
from __future__ import annotations

import sys
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

EXPECTED_TOTAL_MASS = 2.0 + 4 * 0.016076923076923075
PEGASUS_BODIES = ["body", "rotor0", "rotor1", "rotor2", "rotor3"]
PEGASUS_JOINTS = ["joint0", "joint1", "joint2", "joint3"]


def main() -> int:
    default = Path(__file__).resolve().parent.parent / "x500" / "x500.usda"
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    stage = Usd.Stage.Open(str(path))
    errors: list[str] = []

    def check(ok: bool, message: str):
        print(("  ok   " if ok else "  FAIL ") + message)
        if not ok:
            errors.append(message)

    root = stage.GetDefaultPrim()
    print(f"{path}")
    check(bool(root), "defaultPrim is set (needed to reference the file without a prim path)")
    check(UsdGeom.GetStageUpAxis(stage) == "Z", "upAxis is Z")
    check(UsdGeom.GetStageMetersPerUnit(stage) == 1.0, "metersPerUnit is 1")

    everything = list(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))

    roots = [p for p in everything if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    check(len(roots) == 1 and roots[0] == root, f"exactly one ArticulationRootAPI, on the default prim ({[str(p.GetPath()) for p in roots]})")

    bodies = [p for p in everything if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    names = sorted(p.GetName() for p in bodies)
    check(names == sorted(PEGASUS_BODIES), f"rigid bodies are {PEGASUS_BODIES} directly under the root (Pegasus naming)")
    check(all(p.GetParent() == root for p in bodies), "no rigid body is nested inside another")

    total = 0.0
    for body in bodies:
        mass_api = UsdPhysics.MassAPI(body)
        mass = mass_api.GetMassAttr().Get() or 0.0
        inertia = mass_api.GetDiagonalInertiaAttr().Get()
        total += mass
        check(body.HasAPI(UsdPhysics.MassAPI) and mass > 0 and inertia is not None and min(inertia) > 0,
              f"{body.GetName()}: explicit mass {mass:.4f} kg and positive inertia")
        colliders = [c for c in Usd.PrimRange(body, Usd.TraverseInstanceProxies()) if c.HasAPI(UsdPhysics.CollisionAPI)]
        check(len(colliders) > 0, f"{body.GetName()}: has {len(colliders)} collider(s)")
        check(all(c.GetTypeName() in ("Cube", "Cylinder", "Sphere", "Capsule") for c in colliders),
              f"{body.GetName()}: colliders are analytic shapes (no mesh cooking)")
    check(abs(total - EXPECTED_TOTAL_MASS) < 1e-6, f"total mass {total:.4f} kg matches the PX4 model")

    joints = [p for p in everything if p.IsA(UsdPhysics.Joint)]
    check(sorted(p.GetName() for p in joints) == PEGASUS_JOINTS, f"joints are {PEGASUS_JOINTS}")
    body_paths = {p.GetPath() for p in bodies}
    reached = {root.GetPath().AppendChild("body")}
    cache = UsdGeom.XformCache()
    for joint in joints:
        j = UsdPhysics.RevoluteJoint(joint)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        ok = len(b0) == 1 and len(b1) == 1 and b0[0] in body_paths and b1[0] in body_paths
        check(ok, f"{joint.GetName()}: body0/body1 both resolve to rigid bodies")
        if not ok:
            continue
        reached.add(b1[0])
        # the joint frame must coincide in world space when seen from either body
        w0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(b0[0])).Transform(j.GetLocalPos0Attr().Get())
        w1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(b1[0])).Transform(j.GetLocalPos1Attr().Get())
        check((w0 - w1).GetLength() < 1e-6, f"{joint.GetName()}: joint frames agree (no snap at first physics step)")
        check(j.GetAxisAttr().Get() == "Z", f"{joint.GetName()}: spins about Z")
        check(not j.GetLowerLimitAttr().HasAuthoredValue(), f"{joint.GetName()}: continuous (no limits)")
    check(reached == body_paths, "every body is connected to 'body' through the joints")

    # composition and assets
    meshes = [p for p in everything if p.IsA(UsdGeom.Mesh)]
    check(len(meshes) > 0, f"{len(meshes)} visual meshes compose in from geometries.usdc")
    unbound = [str(m.GetPath()) for m in meshes if not UsdShade.MaterialBindingAPI(m).ComputeBoundMaterial()[0]]
    check(not unbound, f"every mesh has a material {unbound[:3] if unbound else ''}")
    check(not any(m.HasAPI(UsdPhysics.CollisionAPI) for m in meshes), "visual meshes carry no collision")
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
    check(not unresolved, f"all references and textures resolve ({len(layers)} layers, {len(assets)} textures) {unresolved or ''}")

    # size: an X500 has a 500 mm wheelbase (diagonal motor-to-motor distance)
    pos = {b.GetName(): cache.GetLocalToWorldTransform(b).ExtractTranslation() for b in bodies}
    if all(k in pos for k in PEGASUS_BODIES):
        wheelbase = max((pos["rotor0"] - pos["rotor1"]).GetLength(), (pos["rotor2"] - pos["rotor3"]).GetLength())
        check(abs(wheelbase - 0.5) < 0.02, f"wheelbase {wheelbase * 1000:.0f} mm (nominal 500)")
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(root).ComputeAlignedRange()
    print(f"  info visual bounds min {tuple(round(v, 3) for v in bbox.GetMin())} max {tuple(round(v, 3) for v in bbox.GetMax())}")
    check(0.25 < bbox.GetSize()[2] < 0.40, f"height {bbox.GetSize()[2]:.3f} m")

    print("\nFAILED" if errors else "\nall checks passed")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
