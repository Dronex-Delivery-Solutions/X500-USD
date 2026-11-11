#!/usr/bin/env python3
"""Build the Holybro X500 USD asset for Isaac Sim from the PX4 Gazebo model.

Source of truth: PX4/PX4-gazebo-models, models/x500 + models/x500_base
(BSD-3-Clause). All poses, masses, inertias, collision boxes and motor
constants below are copied from those two model.sdf files.

Needs only:  pip install usd-core pycollada numpy
(no Isaac Sim required to build).

    python scripts/build_x500.py            # downloads sources into .cache/
    python scripts/build_x500.py --src DIR  # use a local copy of the PX4 files

Output:
    x500/x500.usda          entry point: hierarchy, physics, joints (ASCII)
    x500/geometries.usdc    visual meshes + materials (binary)
    x500/textures/*.png
"""
from __future__ import annotations

import argparse
import math
import shutil
import urllib.request
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

REPO = Path(__file__).resolve().parent.parent
RAW = "https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/models/x500_base/"
SOURCES = {
    "NXP-HGD-CF.dae": "meshes/NXP-HGD-CF.dae",
    "5010Base.dae": "meshes/5010Base.dae",
    "5010Bell.dae": "meshes/5010Bell.dae",
    "1345_prop_ccw.stl": "meshes/1345_prop_ccw.stl",
    "1345_prop_cw.stl": "meshes/1345_prop_cw.stl",
    "CF.png": "meshes/CF.png",
    "nxp.png": "materials/textures/nxp.png",
    "rd.png": "materials/textures/rd.png",
}

# --------------------------------------------------------------------------
# Numbers from x500_base/model.sdf and x500/model.sdf
# --------------------------------------------------------------------------
BODY_MASS = 2.0
BODY_INERTIA = (0.02166666666666667, 0.02166666666666667, 0.04000000000000001)

ROTOR_MASS = 0.016076923076923075
ROTOR_INERTIA = (3.8464910483993325e-07, 2.6115851691700804e-05, 2.649858234714004e-05)
ROTOR_Z = 0.06
ARM = 0.174
# PX4 quad-X numbering. +1 = counter-clockwise seen from above, -1 = clockwise.
ROTORS = [
    # index, x, y, spin, description
    (0, +ARM, -ARM, +1, "front right"),
    (1, -ARM, +ARM, +1, "rear left"),
    (2, +ARM, +ARM, -1, "front left"),
    (3, -ARM, -ARM, -1, "rear right"),
]
PROP_SCALE = 0.8461538461538461
PROP_OFFSET = (-0.022, -0.14638461538461536, -0.016)
BELL_OFFSET = (0.0, 0.0, -0.032)
# The SDF collides each rotor with a 0.279 x 0.017 x 0.0008 m box. A box that
# thin is fragile in PhysX, so the rotor collider is the disc that box sweeps.
ROTOR_DISC_RADIUS = 0.2792307692307692 / 2
ROTOR_DISC_HEIGHT = 0.005

MOTOR = {  # gz MulticopterMotorModel parameters, identical for all 4 motors
    "motorConstant": 8.54858e-06,          # thrust [N] = motorConstant * omega^2
    "momentConstant": 0.016,               # yaw torque [Nm] = momentConstant * thrust
    "maxRotVelocity": 1000.0,              # rad/s
    "rotorDragCoefficient": 8.06428e-05,
    "rollingMomentCoefficient": 1e-06,
    "timeConstantUp": 0.0125,              # s
    "timeConstantDown": 0.025,             # s
}

FRAME_POSE = ((0.0, 0.0, 0.025), (0.0, 0.0, math.pi))
MOTOR_BASES = [
    ((+ARM, +ARM, 0.032), (0, 0, -0.45)),
    ((-ARM, +ARM, 0.032), (0, 0, -0.45)),
    ((+ARM, -ARM, 0.032), (0, 0, -0.45)),
    ((-ARM, -ARM, 0.032), (0, 0, -0.45)),
]
# name, xyz, rpy, plane size (x, y), texture
DECALS = [
    ("NXP_FMUK66_FRONT", (0.047, 0.001, 0.043), (1, 0, 1.57), (0.013, 0.007), "nxp.png"),
    ("NXP_FMUK66_TOP", (-0.023, 0, 0.0515), (0, 0, -1.57), (0.013, 0.007), "nxp.png"),
    ("RDDRONE_FMUK66_TOP", (-0.03, 0, 0.0515), (0, 0, -1.57), (0.032, 0.0034), "rd.png"),
]
# name, xyz, rpy, box size
BODY_COLLIDERS = [
    ("center_plate", (0, 0, 0.007), (0, 0, 0), (0.35355339059327373, 0.35355339059327373, 0.05)),
    ("leg_right", (0, -0.098, -0.123), (-0.35, 0, 0), (0.015, 0.015, 0.21)),
    ("leg_left", (0, 0.098, -0.123), (0.35, 0, 0), (0.015, 0.015, 0.21)),
    ("skid_right", (0, -0.132, -0.2195), (0, 0, 0), (0.25, 0.015, 0.015)),
    ("skid_left", (0, 0.132, -0.2195), (0, 0, 0), (0.25, 0.015, 0.015)),
]

# Lambert colours in the DAE files carry no roughness/metalness; these are
# hand-picked so the frame does not look like plastic everywhere.
SURFACE_HINTS = {
    "CarbonFiber": (0.35, 0.0),
    "Metal": (0.35, 1.0),
    "LandingRubber": (0.9, 0.0),
    "RailsRubber": (0.9, 0.0),
    "FMURubber": (0.8, 0.0),
    "LandingFoam": (1.0, 0.0),
    "MotorMetal": (0.4, 1.0),
    "MotorDark": (0.5, 0.6),
    "Propeller": (0.45, 0.0),
}
DEFAULT_SURFACE = (0.6, 0.0)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def fetch_sources(cache: Path) -> Path:
    cache.mkdir(parents=True, exist_ok=True)
    for name, rel in SOURCES.items():
        dst = cache / name
        if not dst.exists():
            print(f"downloading {rel}")
            urllib.request.urlretrieve(RAW + rel, dst)
    return cache


def rpy_quat(rpy) -> Gf.Quatd:
    """SDF roll-pitch-yaw (fixed axes X, Y, Z) to a quaternion."""
    r, p, y = rpy
    # Gf.Rotation composes left-to-right: the leftmost rotation is applied first.
    q = (
        Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(r))
        * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(p))
        * Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(y))
    )
    return q.GetQuat()


def rpy_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def set_pose(prim, xyz=(0, 0, 0), quat=Gf.Quatd(1, 0, 0, 0), scale=(1, 1, 1)):
    """Author the translate / orient / scale op stack Isaac Sim expects."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*xyz))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))


def weld(points, normals, pidx, nidx, uvs=None, uvidx=None):
    """Collapse per-corner (position, normal, uv) index tuples into unique vertices."""
    cols = [pidx.reshape(-1), nidx.reshape(-1)]
    if uvs is not None:
        cols.append(uvidx.reshape(-1))
    keys = np.stack(cols, axis=1)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    out_uv = uvs[uniq[:, 2]] if uvs is not None else None
    return points[uniq[:, 0]], normals[uniq[:, 1]], inverse.reshape(-1), out_uv


def define_mesh(stage, path, points, normals, indices, uvs=None, double_sided=False):
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = np.asarray(points, dtype=np.float32)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(indices, dtype=np.int32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(len(indices) // 3, 3))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(normals, dtype=np.float32)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(axis=0).tolist()), Gf.Vec3f(*pts.max(axis=0).tolist())])
    if double_sided:
        mesh.CreateDoubleSidedAttr(True)
    if uvs is not None:
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
        )
        st.Set(Vt.Vec2fArray.FromNumpy(np.asarray(uvs, dtype=np.float32)))
    return mesh


def define_material(stage, path, name, color=None, texture=None):
    """UsdPreviewSurface material: renders in Isaac Sim without any MDL."""
    roughness, metallic = SURFACE_HINTS.get(name, DEFAULT_SURFACE)
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    diffuse = shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    if texture:
        reader = UsdShade.Shader.Define(stage, path.AppendChild("stReader"))
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
        tex = UsdShade.Shader.Define(stage, path.AppendChild("diffuseTexture"))
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f"./textures/{texture}"))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        )
        diffuse.ConnectToSource(tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
    else:
        diffuse.Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def bind(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def read_stl(path: Path):
    """Binary STL -> welded points, smooth-ish normals, triangle indices."""
    raw = path.read_bytes()
    count = int(np.frombuffer(raw, dtype="<u4", count=1, offset=80)[0])
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    tris = np.frombuffer(raw, dtype=rec, count=count, offset=84)
    corners = tris["v"].reshape(-1, 3).astype(np.float64)
    points, indices = np.unique(np.round(corners, 7), axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    a, b, c = points[faces[:, 0]], points[faces[:, 1]], points[faces[:, 2]]
    face_n = np.cross(b - a, c - a)  # length = 2 * area, so this area-weights
    normals = np.zeros_like(points)
    for k in range(3):
        np.add.at(normals, faces[:, k], face_n)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals /= np.where(length > 0, length, 1)
    return points, normals, faces.reshape(-1)


# --------------------------------------------------------------------------
# geometries.usdc
# --------------------------------------------------------------------------
def collada_prototype(stage, root, src: Path, rename=None, surface=None):
    """Convert one .dae into root/<mesh> children with their own root/Looks."""
    import collada

    dae = collada.Collada(str(src))
    assert dae.assetInfo.upaxis == "Z_UP" and dae.assetInfo.unitmeter == 1.0, src
    looks = root.GetPath().AppendChild("Looks")
    UsdGeom.Scope.Define(stage, looks)
    tris_total = 0
    for bound in dae.scene.objects("geometry"):
        for prim in bound.primitives():
            if type(prim).__name__ != "BoundTriangleSet" or len(prim) == 0:
                continue  # the frame DAE has a stray line set
            effect = prim.material.effect
            textured = not isinstance(effect.diffuse, tuple)
            name = (rename or {}).get(prim.material.name, prim.material.name)
            name = Tf.MakeValidIdentifier(name)
            pts, nrm, idx, uv = weld(
                prim.vertex,
                prim.normal,
                prim.vertex_index,
                prim.normal_index,
                prim.texcoordset[0] if textured else None,
                prim.texcoord_indexset[0] if textured else None,
            )
            mesh = define_mesh(stage, root.GetPath().AppendChild(name), pts, nrm, idx, uv)
            mat = define_material(
                stage,
                looks.AppendChild(name),
                (surface or {}).get(name, name),
                color=None if textured else effect.diffuse[:3],
                texture=Path(effect.diffuse.sampler.surface.image.path).name if textured else None,
            )
            bind(mesh.GetPrim(), mat)
            tris_total += len(prim)
    return tris_total


def build_geometries(src: Path, out: Path):
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Scope.Define(stage, "/Geometries").GetPrim()
    stage.SetDefaultPrim(root)
    stats = {}

    frame = UsdGeom.Xform.Define(stage, "/Geometries/frame").GetPrim()
    stats["frame"] = collada_prototype(stage, frame, src / "NXP-HGD-CF.dae")

    base = UsdGeom.Xform.Define(stage, "/Geometries/motor_base").GetPrim()
    stats["motor_base"] = collada_prototype(
        stage, base, src / "5010Base.dae", rename={"Material.003": "stator"}, surface={"stator": "MotorMetal"}
    )

    bell = UsdGeom.Xform.Define(stage, "/Geometries/motor_bell").GetPrim()
    stats["motor_bell"] = collada_prototype(
        stage,
        bell,
        src / "5010Bell.dae",
        rename={"Material.002": "bell_cap", "Material.001": "bell"},
        surface={"bell_cap": "MotorDark", "bell": "MotorMetal"},
    )

    # Propellers: bake the SDF scale and offset so the hub sits on the rotor
    # origin and the visual prim needs no transform of its own.
    for tag in ("ccw", "cw"):
        prop = UsdGeom.Xform.Define(stage, f"/Geometries/prop_{tag}").GetPrim()
        pts, nrm, idx = read_stl(src / f"1345_prop_{tag}.stl")
        pts = pts * PROP_SCALE + np.array(PROP_OFFSET)
        mesh = define_mesh(stage, prop.GetPath().AppendChild("blade"), pts, nrm, idx)
        UsdGeom.Scope.Define(stage, prop.GetPath().AppendChild("Looks"))
        mat = define_material(
            stage, prop.GetPath().AppendPath("Looks/blade"), "Propeller", color=(0.03, 0.03, 0.03)
        )
        bind(mesh.GetPrim(), mat)
        stats[f"prop_{tag}"] = len(idx) // 3

    # Logo decals: three tiny textured quads, poses baked into the points.
    decals = UsdGeom.Xform.Define(stage, "/Geometries/decals").GetPrim()
    UsdGeom.Scope.Define(stage, "/Geometries/decals/Looks")
    mats = {}
    for name, xyz, rpy, (sx, sy), tex in DECALS:
        quad = np.array([[-sx / 2, -sy / 2, 0], [sx / 2, -sy / 2, 0], [sx / 2, sy / 2, 0], [-sx / 2, sy / 2, 0]])
        rot = rpy_matrix(rpy)
        pts = quad @ rot.T + np.array(xyz)
        nrm = np.tile(rot @ np.array([0, 0, 1.0]), (4, 1))
        uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
        mesh = define_mesh(
            stage, decals.GetPath().AppendChild(name), pts, nrm, np.array([0, 1, 2, 0, 2, 3]), uv, double_sided=True
        )
        key = Path(tex).stem
        if key not in mats:
            mats[key] = define_material(
                stage, decals.GetPath().AppendPath(f"Looks/{key}"), "decal", texture=tex
            )
        bind(mesh.GetPrim(), mats[key])

    stage.GetRootLayer().Save()
    return stats


# --------------------------------------------------------------------------
# x500.usda
# --------------------------------------------------------------------------
def add_visual(stage, path, proto, xyz=(0, 0, 0), quat=Gf.Quatd(1, 0, 0, 0)):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.GetReferences().AddReference("./geometries.usdc", f"/Geometries/{proto}")
    prim.SetInstanceable(True)
    set_pose(prim, xyz, quat)
    return prim


def add_collider(stage, path, shape, xyz=(0, 0, 0), quat=Gf.Quatd(1, 0, 0, 0), scale=(1, 1, 1)):
    shape.CreatePurposeAttr(UsdGeom.Tokens.guide)  # hidden in renders, visible in debug views
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    set_pose(shape.GetPrim(), xyz, quat, scale)


def add_mass(prim, mass, inertia):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(mass)
    api.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    api.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))


def build_robot(out: Path):
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.GetRootLayer().comment = (
        "Holybro X500 quadrotor for Isaac Sim. Generated by scripts/build_x500.py from "
        "PX4/PX4-gazebo-models (BSD-3-Clause) - edit the script, not this file."
    )

    root = UsdGeom.Xform.Define(stage, "/x500").GetPrim()
    stage.SetDefaultPrim(root)
    root.SetMetadata("kind", "component")
    root.SetDocumentation(
        "Holybro X500. Origin = base_link of the PX4 model (body frame, x forward, y left, z up). "
        "The landing skids reach z = -0.227 m."
    )
    UsdPhysics.ArticulationRootAPI.Apply(root)

    # ---- airframe ---------------------------------------------------------
    body = UsdGeom.Xform.Define(stage, "/x500/body").GetPrim()
    set_pose(body)
    add_mass(body, BODY_MASS, BODY_INERTIA)
    UsdGeom.Scope.Define(stage, "/x500/body/visuals")
    add_visual(stage, "/x500/body/visuals/frame", "frame", FRAME_POSE[0], rpy_quat(FRAME_POSE[1]))
    add_visual(stage, "/x500/body/visuals/decals", "decals")
    for i, (xyz, rpy) in enumerate(MOTOR_BASES):
        add_visual(stage, f"/x500/body/visuals/motor_base_{i}", "motor_base", xyz, rpy_quat(rpy))
    UsdGeom.Scope.Define(stage, "/x500/body/collisions")
    for name, xyz, rpy, size in BODY_COLLIDERS:
        cube = UsdGeom.Cube.Define(stage, f"/x500/body/collisions/{name}")
        cube.CreateSizeAttr(1.0)
        cube.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
        add_collider(stage, cube.GetPath(), cube, xyz, rpy_quat(rpy), size)

    # ---- rotors and joints -------------------------------------------------
    UsdGeom.Scope.Define(stage, "/x500/joints")
    for i, x, y, spin, where in ROTORS:
        rotor = UsdGeom.Xform.Define(stage, f"/x500/rotor{i}").GetPrim()
        rotor.SetDocumentation(f"Rotor {i} ({where}), spins {'counter-' if spin > 0 else ''}clockwise seen from above.")
        set_pose(rotor, (x, y, ROTOR_Z))
        add_mass(rotor, ROTOR_MASS, ROTOR_INERTIA)
        UsdGeom.Scope.Define(stage, f"/x500/rotor{i}/visuals")
        add_visual(stage, f"/x500/rotor{i}/visuals/propeller", "prop_ccw" if spin > 0 else "prop_cw")
        add_visual(stage, f"/x500/rotor{i}/visuals/motor_bell", "motor_bell", BELL_OFFSET)
        UsdGeom.Scope.Define(stage, f"/x500/rotor{i}/collisions")
        disc = UsdGeom.Cylinder.Define(stage, f"/x500/rotor{i}/collisions/disc")
        disc.CreateAxisAttr(UsdGeom.Tokens.z)
        disc.CreateRadiusAttr(ROTOR_DISC_RADIUS)
        disc.CreateHeightAttr(ROTOR_DISC_HEIGHT)
        r, h = ROTOR_DISC_RADIUS, ROTOR_DISC_HEIGHT / 2
        disc.CreateExtentAttr([(-r, -r, -h), (r, r, h)])
        add_collider(stage, disc.GetPath(), disc)

        # Motor model parameters travel with the asset as plain custom attributes.
        rotor.CreateAttribute("x500:motor:spinDirection", Sdf.ValueTypeNames.Int).Set(spin)
        for key, value in MOTOR.items():
            rotor.CreateAttribute(f"x500:motor:{key}", Sdf.ValueTypeNames.Double).Set(value)

        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/x500/joints/joint{i}")
        joint.CreateBody0Rel().SetTargets(["/x500/body"])
        joint.CreateBody1Rel().SetTargets([f"/x500/rotor{i}"])
        joint.CreateAxisAttr(UsdPhysics.Tokens.z)
        joint.CreateLocalPos0Attr(Gf.Vec3f(x, y, ROTOR_Z))
        joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        # no limits authored: the rotor joint is continuous

    stage.GetRootLayer().Save()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=Path, help="folder holding the PX4 x500_base meshes and textures (flat)")
    ap.add_argument("--out", type=Path, default=REPO / "x500")
    args = ap.parse_args()

    src = args.src or fetch_sources(REPO / ".cache" / "px4_x500_base")
    out = args.out
    (out / "textures").mkdir(parents=True, exist_ok=True)
    for tex in ("CF.png", "nxp.png", "rd.png"):
        shutil.copyfile(src / tex, out / "textures" / tex)

    stats = build_geometries(src, out / "geometries.usdc")
    build_robot(out / "x500.usda")
    print("triangles:", stats, "total", sum(stats.values()))
    print("wrote", out / "x500.usda")


if __name__ == "__main__":
    main()
