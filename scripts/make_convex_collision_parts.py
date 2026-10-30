#!/usr/bin/env python3
"""Cut a concave collision mesh into convex pieces for MuJoCo.

MuJoCo collides a mesh geom as its convex hull. For a concave object, such as a door lever,
the hull fills the gaps in the shape and contact happens in empty space. This script cuts the
mesh with axis-aligned planes and writes the convex hull of each piece as
``<stem>_convex_<i>.stl`` beside ``<stem>.stl``. The hull of a small piece follows the surface
much more closely than the hull of the whole mesh.

Each ``--piece`` is one box of limits on the mesh's own axes, for example ``"y<-0.0175"`` or
``"y>0.014,0.025<x<0.045"``. The pieces should cover the whole mesh and not overlap. Run with
only ``--report`` first to see the mesh bounds and how far the single hull is from the surface,
then add pieces until the report is close enough. Write ``--piece="..."`` with ``=``: a limit
that starts with ``-`` otherwise reads as an option.

Example, the door lever:

    python3 scripts/make_convex_collision_parts.py \\
      ihmc-high-level-behaviors/src/libgdx/resources/environmentObjects/door_handle/door_handle_collision.stl \\
      --piece="y<-0.0175" --piece="-0.0175<y<0.014" \\
      --piece="y>0.014,x<0.025" --piece="y>0.014,0.025<x<0.045" --piece="y>0.014,0.045<x<0.065" \\
      --piece="y>0.014,0.065<x<0.085" --piece="y>0.014,x>0.085"
"""

import argparse
import re
import struct
import sys
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import ConvexHull, cKDTree

AXES = {"x": 0, "y": 1, "z": 2}
NUMBER = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
BETWEEN = re.compile(rf"^({NUMBER})<([xyz])<({NUMBER})$")
BELOW = re.compile(rf"^([xyz])<({NUMBER})$")
ABOVE = re.compile(rf"^([xyz])>({NUMBER})$")


def parse_piece(spec: str) -> dict[int, tuple[float | None, float | None]]:
    """``"y>0.014,0.025<x<0.045"`` -> ``{1: (0.014, None), 0: (0.025, 0.045)}``."""
    box = {}
    for condition in spec.replace(" ", "").split(","):
        if match := BETWEEN.match(condition):
            low, axis, high = float(match[1]), match[2], float(match[3])
        elif match := BELOW.match(condition):
            low, axis, high = None, match[1], float(match[2])
        elif match := ABOVE.match(condition):
            low, axis, high = float(match[2]), match[1], None
        else:
            sys.exit(f"Cannot read piece condition '{condition}'. Use x<a, x>a, or a<x<b.")
        if AXES[axis] in box:
            sys.exit(f"Piece '{spec}' limits {axis} twice")
        box[AXES[axis]] = (low, high)
    return box


def piece_points(triangles: np.ndarray, box: dict) -> np.ndarray:
    """Mesh vertices inside the box, plus where mesh edges cross its faces. Their hull is the piece's hull."""
    edges = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    points = [edges.reshape(-1, 3)]
    for axis, limits in box.items():
        for plane in limits:
            if plane is None:
                continue
            a, b = edges[:, 0], edges[:, 1]
            da, db = a[:, axis] - plane, b[:, axis] - plane
            crossing = da * db < 0
            t = da[crossing] / (da[crossing] - db[crossing])
            points.append(a[crossing] + t[:, None] * (b[crossing] - a[crossing]))
    points = np.concatenate(points)
    keep = np.ones(len(points), bool)
    for axis, (low, high) in box.items():
        if low is not None:
            keep &= points[:, axis] >= low - 1e-9
        if high is not None:
            keep &= points[:, axis] <= high + 1e-9
    return points[keep]


def in_hull(hull: ConvexHull, points: np.ndarray) -> np.ndarray:
    return np.all(points @ hull.equations[:, :3].T + hull.equations[:, 3] <= 1e-9, axis=1)


def in_mesh(triangles: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Ray parity along +x. Needs a closed mesh."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    v0, v1 = c[:, 1:] - a[:, 1:], b[:, 1:] - a[:, 1:]
    d00, d01, d11 = (v0 * v0).sum(1), (v0 * v1).sum(1), (v1 * v1).sum(1)
    denominator = d00 * d11 - d01 * d01
    denominator[denominator == 0] = 1e-30
    inside = np.zeros(len(points), bool)
    chunk = max(1, 3_000_000 // len(triangles))
    for start in range(0, len(points), chunk):
        query = points[start:start + chunk]
        v2 = query[:, None, 1:] - a[None, :, 1:]
        d20, d21 = (v2 * v0).sum(2), (v2 * v1).sum(2)
        u = (d11 * d20 - d01 * d21) / denominator
        w = (d00 * d21 - d01 * d20) / denominator
        hit = (u >= 0) & (w >= 0) & (u + w <= 1)
        hit_x = a[:, 0] + u * (c[:, 0] - a[:, 0]) + w * (b[:, 0] - a[:, 0])
        inside[start:start + chunk] = ((hit & (hit_x > query[:, None, 0])).sum(1) % 2) == 1
    return inside


def report(label: str, hulls: list[ConvexHull], triangles: np.ndarray, samples: int) -> None:
    """How much the hulls add outside the mesh, and how deep, and whether they miss any of it."""
    vertices = triangles.reshape(-1, 3)
    low, high = vertices.min(0) - 0.003, vertices.max(0) + 0.003
    rng = np.random.default_rng(0)
    query = rng.uniform(low, high, (samples, 3))
    inside_mesh = in_mesh(triangles, query)
    weights = rng.dirichlet((1, 1, 1), len(triangles) * 4)
    surface = (np.repeat(triangles, 4, axis=0) * weights[:, :, None]).sum(1)
    surface_tree = cKDTree(np.vstack([surface, vertices]))

    print(f"{label}:")
    inside_any = np.zeros(samples, bool)
    for index, hull in enumerate(hulls):
        inside = in_hull(hull, query)
        inside_any |= inside
        extra = query[inside & ~inside_mesh]
        depth = surface_tree.query(extra)[0] * 1000.0 if len(extra) else np.zeros(1)
        name = f"piece {index}" if len(hulls) > 1 else "hull"
        print(f"  {name:9s} {len(hull.simplices):5d} faces   outside the mesh: median {np.median(depth):4.1f} mm, max {depth.max():4.1f} mm")
    extra_fraction = (inside_any & ~inside_mesh).sum() / max(inside_mesh.sum(), 1)
    missed_fraction = (inside_mesh & ~inside_any).sum() / max(inside_mesh.sum(), 1)
    print(f"  volume outside the mesh: {extra_fraction * 100:.0f}% of the mesh volume. Mesh not covered: {missed_fraction * 100:.1f}%")
    if missed_fraction > 0.001:
        print("  warning: the pieces leave part of the mesh uncovered. Check that the boxes meet.")


def write_stl(path: Path, hull: ConvexHull) -> None:
    with open(path, "wb") as file:
        file.write(b"convex collision piece".ljust(80, b" "))
        file.write(struct.pack("<I", len(hull.simplices)))
        for simplex, equation in zip(hull.simplices, hull.equations):
            triangle = hull.points[simplex]
            normal = equation[:3]
            if np.dot(np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0]), normal) < 0:
                triangle = triangle[[0, 2, 1]]
            file.write(struct.pack("<3f", *normal))
            file.write(struct.pack("<9f", *triangle.ravel()))
            file.write(b"\0\0")


def main() -> None:
    parser = argparse.ArgumentParser(description="Cut a concave collision mesh into convex pieces for MuJoCo")
    parser.add_argument("mesh", help="Closed collision mesh, usually <stem>_collision.stl")
    parser.add_argument("--piece", action="append", default=[], help='Box of axis limits, e.g. --piece="y>0.014,0.025<x<0.045". Repeat for each piece.')
    parser.add_argument("--report", action="store_true", help="Print how close the hulls are to the mesh")
    parser.add_argument("--samples", type=int, default=20000, help="Sample points for --report (default 20000)")
    parser.add_argument("--dry-run", action="store_true", help="Print what would be written without writing")
    args = parser.parse_args()

    source = Path(args.mesh)
    mesh = trimesh.load(source, force="mesh")
    if not mesh.is_watertight:
        print("warning: the mesh is not closed, so the --report volumes are unreliable")
    triangles = np.asarray(mesh.triangles, dtype=float)
    print(f"{source.name}: {len(triangles)} triangles, bounds min {np.round(mesh.bounds[0], 4)} max {np.round(mesh.bounds[1], 4)}")

    if args.report or not args.piece:
        report("Single convex hull (what MuJoCo uses for the mesh as is)", [ConvexHull(triangles.reshape(-1, 3))], triangles, args.samples)
    if not args.piece:
        return

    hulls = []
    for spec in args.piece:
        points = piece_points(triangles, parse_piece(spec))
        if len(points) < 4:
            sys.exit(f"Piece '{spec}' holds no part of the mesh")
        hulls.append(ConvexHull(points))
    if args.report:
        report(f"{len(hulls)} convex pieces", hulls, triangles, args.samples)

    # The loader reads <stem>_convex_0.stl, _1, ... up to the first missing index, so clear old pieces first.
    stale = sorted(source.parent.glob(f"{source.stem}_convex_*.stl"))
    outputs = [source.with_name(f"{source.stem}_convex_{index}.stl") for index in range(len(hulls))]
    if args.dry_run:
        for path in stale:
            print(f"  would remove {path.name}")
        for path, hull in zip(outputs, hulls):
            print(f"  would write {path.name} ({len(hull.simplices)} faces)")
        return
    for path in stale:
        path.unlink()
    for path, hull in zip(outputs, hulls):
        write_stl(path, hull)
        print(f"  wrote {path.name} ({len(hull.simplices)} faces)")


if __name__ == "__main__":
    main()
