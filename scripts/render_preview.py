#!/usr/bin/env python3
"""Software preview of the composed asset - no GPU, no Isaac Sim.

    pip install usd-core numpy pillow
    python scripts/render_preview.py            # writes docs/preview.png

Top row: visual meshes. Bottom row: collision shapes (orange) over the visuals.
It is a crude point-splat renderer meant for checking that parts are in the
right place; it says nothing about how the materials look under RTX.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

REPO = Path(__file__).resolve().parent.parent
RES = 560
HALF_EXTENT = 0.40  # metres shown from the image centre to its edge
VIEWS = {  # name: direction the camera looks along, camera "up"
    "front (looking -x)": ((-1, 0, 0), (0, 0, 1)),
    "left side (looking -y)": ((0, -1, 0), (0, 0, 1)),
    "three-quarter": ((-1, -0.8, -0.7), (0, 0, 1)),
    "top (x forward = up)": ((0, 0, -1), (1, 0, 0)),
}
CENTER = np.array([0.0, 0.0, -0.09])
BACKGROUND = np.array([236, 238, 242], dtype=np.float32)
rng = np.random.default_rng(0)


def material_color(prim) -> np.ndarray:
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if mat:
        shader = mat.ComputeSurfaceSource()[0]
        inp = shader.GetInput("diffuseColor") if shader else None
        if inp and not inp.HasConnectedSource() and inp.Get() is not None:
            return np.array(inp.Get(), dtype=np.float32)
        if inp and inp.HasConnectedSource():
            return np.array([0.07, 0.07, 0.08], dtype=np.float32)  # carbon-fibre texture stand-in
    return np.array([0.5, 0.5, 0.5], dtype=np.float32)


def box_tris():
    v = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    f = [(a, b, c) for a, b, c, d in quads] + [(a, c, d) for a, b, c, d in quads]
    return v, np.array(f)


def cylinder_tris(radius, height, n=48):
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang)], axis=1)
    top = np.c_[ring, np.full(n, height / 2)]
    bot = np.c_[ring, np.full(n, -height / 2)]
    v = np.vstack([top, bot, [[0, 0, height / 2]], [[0, 0, -height / 2]]])
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [(i, j, 2 * n), (n + j, n + i, 2 * n + 1), (i, n + i, n + j), (i, n + j, j)]
    return v, np.array(f)


def collect(stage):
    """World-space triangles for visuals and for colliders."""
    cache = UsdGeom.XformCache()
    visual, collide = [], []
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        is_collider = prim.HasAPI(UsdPhysics.CollisionAPI)
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            v = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
            f = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        elif prim.IsA(UsdGeom.Cube):
            v, f = box_tris()
            v = v * UsdGeom.Cube(prim).GetSizeAttr().Get()
        elif prim.IsA(UsdGeom.Cylinder):
            cyl = UsdGeom.Cylinder(prim)
            v, f = cylinder_tris(cyl.GetRadiusAttr().Get(), cyl.GetHeightAttr().Get())
        else:
            continue
        m = np.array(cache.GetLocalToWorldTransform(prim))  # row-vector convention
        v = np.c_[v, np.ones(len(v))] @ m
        tris = v[:, :3][f]
        color = np.array([1.0, 0.45, 0.05], dtype=np.float32) if is_collider else material_color(prim)
        (collide if is_collider else visual).append((tris, color))
    return visual, collide


def render(groups, look, up, dim=1.0, canvas=None, depth=None):
    look = np.array(look, dtype=np.float64)
    look /= np.linalg.norm(look)
    right = np.cross(look, np.array(up, dtype=np.float64))
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, look)
    light = -look + 0.5 * cam_up + 0.3 * right
    light /= np.linalg.norm(light)
    px_per_m = RES / (2 * HALF_EXTENT)
    if canvas is None:
        canvas = np.tile(BACKGROUND, (RES, RES, 1))
        depth = np.full((RES, RES), np.inf)
    for tris, color in groups:
        a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
        normal = np.cross(b - a, c - a)
        area = np.linalg.norm(normal, axis=1)
        keep = area > 0
        a, b, c, normal, area = a[keep], b[keep], c[keep], normal[keep], area[keep]
        normal /= area[:, None]
        shade = 0.35 + 0.65 * np.abs(normal @ light)
        # sample each triangle densely enough to leave no holes
        samples = np.maximum(3, np.ceil(area / 2 * px_per_m**2 * 6)).astype(int)
        tri = np.repeat(np.arange(len(a)), samples)
        u, v = rng.random(len(tri)), rng.random(len(tri))
        flip = u + v > 1
        u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
        p = a[tri] + u[:, None] * (b[tri] - a[tri]) + v[:, None] * (c[tri] - a[tri]) - CENTER
        x = np.round(p @ right * px_per_m + RES / 2).astype(int)
        y = np.round(RES / 2 - p @ cam_up * px_per_m).astype(int)
        z = p @ look
        ok = (x >= 0) & (x < RES) & (y >= 0) & (y < RES)
        x, y, z, tri = x[ok], y[ok], z[ok], tri[ok]
        order = np.argsort(-z)  # far first, so the nearest sample is written last
        x, y, z, tri = x[order], y[order], z[order], tri[order]
        nearer = z < depth[y, x]
        x, y, z, tri = x[nearer], y[nearer], z[nearer], tri[nearer]
        rgb = np.clip(color[None, :] * shade[tri][:, None] * dim + (1 - dim), 0, 1) ** (1 / 2.2) * 255
        canvas[y, x] = rgb
        depth[y, x] = z
    return canvas, depth


def main():
    usd = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "x500" / "x500.usda"
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else REPO / "docs" / "preview.png"
    visual, collide = collect(Usd.Stage.Open(str(usd)))
    sheet = Image.new("RGB", (RES * len(VIEWS), RES * 2), "white")
    draw = ImageDraw.Draw(sheet)
    for col, (name, (look, up)) in enumerate(VIEWS.items()):
        top, _ = render(visual, look, up)
        sheet.paste(Image.fromarray(top.astype(np.uint8)), (col * RES, 0))
        bottom, depth = render(visual, look, up, dim=0.35)
        depth[:] = np.inf  # draw colliders on top so enclosed ones stay visible
        bottom, _ = render(collide, look, up, canvas=bottom, depth=depth)
        sheet.paste(Image.fromarray(bottom.astype(np.uint8)), (col * RES, RES))
        draw.text((col * RES + 10, 8), name, fill=(30, 30, 30))
        draw.text((col * RES + 10, RES + 8), name + " - colliders", fill=(30, 30, 30))
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
