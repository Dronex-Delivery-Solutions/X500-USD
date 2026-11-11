"""Shared USD-authoring helpers for build_x500.py and the sensors/*/build_*.py
scripts. Not a public API - internal to this repo's build scripts.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Tf, UsdGeom, UsdPhysics, UsdShade, Vt

# Lambert colours in the DAE files carry no roughness/metalness; these are
# hand-picked so parts do not look like plastic everywhere. Shared across
# every variant so accessories match the base airframe's look.
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
    "Glass": (0.05, 0.0),
    "LensGlass": (0.05, 0.0),
}
DEFAULT_SURFACE = (0.6, 0.0)


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


def matrix_to_quat(matrix: np.ndarray) -> Gf.Quatd:
    """A 3x3 rotation matrix (world = matrix @ local convention, i.e.
    world_vec = matrix @ local_vec, same as rpy_matrix) to a Gf.Quatd.
    Used to combine two chained SDF poses (parent-relative rpy_matrix(a) @
    child-relative rpy_matrix(b)) without relying on Gf.Rotation's own
    multiplication order.

    pxr.Gf uses the row-vector convention (world_vec = local_vec @ M), the
    transpose of the convention used here and in rpy_matrix/numpy
    (world_vec = M @ local_vec) - so the matrix is transposed before
    handing it to Gf.Matrix3d. Verified against Gf.Rotation(q).TransformDir
    for several non-symmetric rotations; get this backwards and every
    *combined* (two-pose-chained) orientation in this repo silently
    becomes its own transpose, which for a generic rotation is a
    different, wrong orientation (it only happens to look right for a
    180-degree rotation, which is its own transpose).
    """
    m = np.asarray(matrix, dtype=float).T
    return Gf.Matrix3d(*m.flatten()).ExtractRotation().GetQuat()


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


def collada_prototype(stage, root, src: Path, rename=None, surface=None, unit_scale=1.0):
    """Convert one .dae into root/<mesh> children with their own root/Looks.

    unit_scale: multiply positions by this (e.g. 0.01 for a COLLADA file
    authored in centimetres, such as PX4's lidar_2d_v2.dae) so every
    prototype ends up in metres regardless of what the source file declares.
    Polygon primitives (polylists) are triangulated automatically.
    """
    import collada

    dae = collada.Collada(str(src))
    assert dae.assetInfo.upaxis == "Z_UP", src
    looks = root.GetPath().AppendChild("Looks")
    UsdGeom.Scope.Define(stage, looks)
    tris_total = 0
    for bound in dae.scene.objects("geometry"):
        for prim in bound.primitives():
            type_name = type(prim).__name__
            if type_name == "BoundPolylist":
                prim = prim.triangleset()
                type_name = "BoundTriangleSet"
            if type_name != "BoundTriangleSet" or len(prim) == 0:
                continue  # stray line sets etc.
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
            if unit_scale != 1.0:
                pts = pts * unit_scale
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


def add_visual(stage, path, ref_file, proto, xyz=(0, 0, 0), quat=Gf.Quatd(1, 0, 0, 0)):
    """Reference a /Geometries/<proto> prototype from ref_file (relative path)."""
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.GetReferences().AddReference(ref_file, f"/Geometries/{proto}")
    prim.SetInstanceable(True)
    set_pose(prim, xyz, quat)
    return prim


def add_collider(stage, path_or_shape, xyz=(0, 0, 0), quat=Gf.Quatd(1, 0, 0, 0), scale=(1, 1, 1)):
    shape = path_or_shape
    shape.CreatePurposeAttr(UsdGeom.Tokens.guide)  # hidden in renders, visible in debug views
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    set_pose(shape.GetPrim(), xyz, quat, scale)


def add_mass(prim, mass, inertia, center_of_mass=(0, 0, 0)):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(mass)
    api.CreateCenterOfMassAttr(Gf.Vec3f(*center_of_mass))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    api.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))


# Isaac/USD cameras look down local -Z with +Y up, +X right (the fixed
# OpenGL-style convention every UsdGeom.Camera uses, regardless of stage
# up-axis). Gazebo/PX4 SDF camera sensors look down the SENSOR FRAME's own
# +X axis with +Z up (consistent with how PX4's camera-equipped x500
# variants give the camera link zero extra rotation to face it forward,
# and an explicit +90deg pitch to face it down) - i.e. the ROS REP-103
# "optical frame" convention, itself related to USD's by a fixed 180deg
# flip about the optical frame's own X axis. FORWARD_CAMERA_ROTATION is the
# constant rotation that, applied to a zero-rotation USD Camera prim,
# makes it look along the body's own +X with +Z up, matching an SDF camera
# sensor mounted with no extra pose. Derived and verified in
# matrix_to_quat's docstring tests; apply the body-frame mount's own extra
# rotation (if the real sensor is pitched down, angled, etc.) on top of it.
FORWARD_CAMERA_ROTATION = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)


def add_camera(stage, path, xyz, focal_length, h_aperture, v_aperture, clip_range, extra_rotation=None):
    """A UsdGeom.Camera aimed at body-forward (+X) by default - pass
    extra_rotation (a 3x3 matrix, applied in body/mount frame, same
    convention as rpy_matrix) for a camera mounted at some other angle,
    e.g. straight down.
    """
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(float(focal_length))
    cam.CreateHorizontalApertureAttr(float(h_aperture))
    cam.CreateVerticalApertureAttr(float(v_aperture))
    cam.CreateClippingRangeAttr(Gf.Vec2f(*clip_range))
    rotation = FORWARD_CAMERA_ROTATION if extra_rotation is None else extra_rotation @ FORWARD_CAMERA_ROTATION
    set_pose(cam.GetPrim(), xyz, matrix_to_quat(rotation))
    return cam


def add_generic_lidar(stage, path, min_range, max_range, azimuth_deg, elevation_deg, scan_period_s,
                       horizontal_fov=None, vertical_fov=0.0, draw_points=True):
    """Author an OmniSensorGenericLidarCoreAPI prim (Isaac Sim 6's custom-scan-
    pattern lidar authoring path, documented in the isaac-sim-sensor skill).

    This schema is a proprietary Kit/Isaac Sim extension, not part of
    OpenUSD - it is not registered in plain `usd-core`, so this function
    cannot validate it the way the UsdPhysics calls elsewhere in this file
    are validated. It authors exactly the attribute names and the
    apiSchemas token the skill documents, as plain (schema-less) USD
    metadata/attributes, which round-trip correctly through any USD writer
    and are picked up by Isaac Sim's own schema plugin at load time. This
    has NOT been confirmed to work by loading it in Isaac Sim.

    azimuth_deg, elevation_deg: one beam direction per array entry (same
    length). fireTimeNs is spread evenly across one scan period so beams
    fire in array order, which is how a mechanically rotating lidar (the
    2D case here) actually produces a frame; for a single fixed beam this
    just puts it at fireTimeNs=0.
    """
    n = len(azimuth_deg)
    assert len(elevation_deg) == n
    fire_time_ns = (np.arange(n) * (scan_period_s * 1e9 / max(n, 1))).astype(np.int64)

    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.AddAppliedSchema("OmniSensorGenericLidarCoreAPI")
    prim.CreateAttribute("minRange", Sdf.ValueTypeNames.Float).Set(float(min_range))
    prim.CreateAttribute("maxRange", Sdf.ValueTypeNames.Float).Set(float(max_range))
    prim.CreateAttribute("horizontalFov", Sdf.ValueTypeNames.Float).Set(
        float(horizontal_fov if horizontal_fov is not None else azimuth_deg.max() - azimuth_deg.min())
    )
    prim.CreateAttribute("horizontalResolution", Sdf.ValueTypeNames.Float).Set(
        float(abs(azimuth_deg[1] - azimuth_deg[0])) if n > 1 else 1.0
    )
    prim.CreateAttribute("verticalFov", Sdf.ValueTypeNames.Float).Set(float(vertical_fov))
    prim.CreateAttribute("verticalResolution", Sdf.ValueTypeNames.Float).Set(1.0)
    prim.CreateAttribute("highLod", Sdf.ValueTypeNames.Bool).Set(True)
    prim.CreateAttribute("drawPoints", Sdf.ValueTypeNames.Bool).Set(bool(draw_points))
    prim.CreateAttribute("omni:sensor:Core:emitterState:s001:azimuthDeg", Sdf.ValueTypeNames.FloatArray).Set(
        Vt.FloatArray.FromNumpy(azimuth_deg.astype(np.float32))
    )
    prim.CreateAttribute("omni:sensor:Core:emitterState:s001:elevationDeg", Sdf.ValueTypeNames.FloatArray).Set(
        Vt.FloatArray.FromNumpy(elevation_deg.astype(np.float32))
    )
    prim.CreateAttribute("omni:sensor:Core:emitterState:s001:fireTimeNs", Sdf.ValueTypeNames.IntArray).Set(
        Vt.IntArray.FromNumpy(fire_time_ns.astype(np.int32))
    )
    return prim


def add_fixed_joint(stage, path, body0, body1, pos0, pos1=(0, 0, 0), rot0=Gf.Quatf(1, 0, 0, 0)):
    """A rigid (non-articulated) weld: used for sensor accessories bolted to
    the airframe, exactly as PX4's SDF joins each sensor model with a
    <joint type="fixed">.
    """
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    joint.CreateLocalRot0Attr(rot0)
    joint.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateExcludeFromArticulationAttr(True)
    return joint
