"""
Convert a Habitat-Matterport 3D (HM3D) scene into an RDX environment object with MuJoCo collision.

HM3D scenes are single scanned meshes. MuJoCo collides every mesh geom as its convex hull, so a
building cannot collide as one mesh: the hull would fill every room. Instead the scan becomes boxes:

* Walkable surfaces (floors, stair treads, furniture tops) are binned at a fine pitch, snapped to
  the dominant floor heights, small scan holes are filled, and each surface is extended down to a
  solid slab so stairs become solid steps. Rugs and mats, flat patches a few centimeters above a floor
  and bordered only by it, are put down onto the floor, so they don't become steps that trip the robot.
* Everything else (walls, furniture) becomes, per story, columns from the floor up to the tallest
  obstacle in each coarse cell that has scan points within the robot's reach. Cells with points only
  overhead (door headers, beams) stay open. Scan points just above or below a nearby walkable surface (stair
  risers, wall bases) are left out so they neither trip the robot nor block a staircase.
* Floors are filled under walls and furniture and across small scan holes, since a slab under a
  wall is harmless, which keeps them a few large boxes. Openings over stairs stay open.
* Everything is merged greedily into boxes, a few thousand per building, which MuJoCo steps cheaply.

The visual GLB is copied with vertex normals added and a gray block exterior on the back of each surface. HM3D GLBs are Z-up in meters, as RDX expects. Outputs go to
``ihmc-high-level-behaviors/src/libgdx/resources/environmentObjects/hm3d/<scene>/``, which is
git-ignored because HM3D is licensed for academic, non-commercial use only:

    <scene>.glb                  visual, with vertex normals added for lighting
    <scene>_collision_boxes.txt  one box per line: sizeX sizeY sizeZ x y z, in the object frame
    <scene>_rdx.json             visual offset, floor heights, bounds

The object frame is centered on the scene footprint, with z = 0 at the lowest floor.
RDXHM3DSceneObject loads the scenes listed in ``environmentObjects/hm3d/scenes.txt``.

Usage, from the ihmc-open-robotics-software repository:
    python3 scripts/hm3d_to_rdx.py ~/datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.glb
Requires numpy, scipy, trimesh and pygltflib.
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import trimesh
from pygltflib import GLTF2, Accessor, Attributes, BufferView, Image, Material, PbrMetallicRoughness, Primitive, Sampler, Texture, TextureInfo
from scipy import ndimage

REPO = Path(__file__).resolve().parent.parent
HM3D_RESOURCES = REPO / "ihmc-high-level-behaviors" / "src" / "libgdx" / "resources" / "environmentObjects" / "hm3d"
WALKABLE_NORMAL_Z = np.cos(np.radians(25.0))


def sample_faces(mesh: trimesh.Trimesh, faces: np.ndarray, spacing: float, rng: np.random.Generator) -> np.ndarray:
    """Points on the given faces, about one per spacing squared."""
    if len(faces) == 0:
        return np.zeros((0, 3))
    areas = mesh.area_faces[faces]
    counts = rng.poisson(areas / (spacing * spacing))
    # Every face gets its centroid, so small triangles are never missed
    chosen = np.repeat(faces, counts)
    triangles = mesh.triangles[chosen]
    u = rng.random((len(chosen), 2))
    flip = u.sum(axis=1) > 1.0
    u[flip] = 1.0 - u[flip]
    points = triangles[:, 0] + u[:, :1] * (triangles[:, 1] - triangles[:, 0]) + u[:, 1:] * (triangles[:, 2] - triangles[:, 0])
    return np.vstack([points, mesh.triangles_center[faces]])


def floor_levels(heights: np.ndarray, weights: np.ndarray, min_area: float, cell_area: float, min_story_height: float) -> list[float]:
    """
    Heights of the floor of each story: the largest flat surfaces, at least a story apart, so large
    beds and tables are not mistaken for floors.
    """
    bins = np.arange(heights.min() - 0.01, heights.max() + 0.02, 0.01)
    histogram, edges = np.histogram(heights, bins=bins, weights=weights)
    # Sum a 5 cm window so scan noise does not split one floor across bins
    window = np.convolve(histogram, np.ones(5), mode="same")
    levels = []
    for i in np.argsort(window)[::-1]:
        if window[i] * cell_area < min_area:
            break
        center = 0.5 * (edges[i] + edges[i + 1])
        if all(abs(center - level) >= min_story_height for level in levels):
            levels.append(center)
    # Refine each level to the weighted mean of nearby heights
    refined = []
    for level in levels:
        near = np.abs(heights - level) < 0.04
        refined.append(float(np.average(heights[near], weights=weights[near])))
    return sorted(refined)


def walkable_surfaces(points: np.ndarray, origin: np.ndarray, pitch: float, shape: tuple[int, int]):
    """
    Per xy cell, the distinct surface heights (top of each cluster of points).
    Returns arrays of cell x, cell y, height and point count.
    """
    cells = np.floor((points[:, :2] - origin[:2]) / pitch).astype(np.int64)
    cell_id = cells[:, 0] * shape[1] + cells[:, 1]
    order = np.lexsort((points[:, 2], cell_id))
    cell_id = cell_id[order]
    z = points[order, 2]
    new_cluster = np.ones(len(z), dtype=bool)
    new_cluster[1:] = (cell_id[1:] != cell_id[:-1]) | (np.diff(z) > 0.05)
    starts = np.flatnonzero(new_cluster)
    tops = np.maximum.reduceat(z, starts)
    counts = np.diff(np.append(starts, len(z)))
    ids = cell_id[starts]
    return ids // shape[1], ids % shape[1], tops, counts


def flatten_patches(cx, cy, top, snapped, shape, args):
    """
    Surfaces off the floor levels (stair treads, furniture tops) are noisy. Group them into connected
    patches of similar height, drop patches too small to step on, and flatten each nearly flat patch
    to its median height so it merges into a few boxes.
    """
    other = ~snapped
    layer = np.round((top[other] - top[other].min()) / args.step_quantum).astype(np.int64)
    grid = np.zeros((*shape, layer.max() + 1), dtype=bool)
    grid[cx[other], cy[other], layer] = True
    labels, count = ndimage.label(grid, structure=np.ones((3, 3, 3)))
    patch = labels[cx[other], cy[other], layer]
    sizes = np.bincount(patch, minlength=count + 1)
    heights = top[other]
    flattened = heights.copy()
    order = np.argsort(patch)
    starts = np.flatnonzero(np.diff(np.append(-1, patch[order])))
    for start, end in zip(starts, np.append(starts[1:], len(order))):
        members = order[start:end]
        values = heights[members]
        if values.max() - values.min() <= args.flat_patch_range:
            flattened[members] = np.median(values)
    flat = np.zeros(len(heights), dtype=bool)
    for start, end in zip(starts, np.append(starts[1:], len(order))):
        members = order[start:end]
        flat[members] = heights[members].max() - heights[members].min() <= args.flat_patch_range
    keep = np.ones(len(top), dtype=bool)
    keep[np.flatnonzero(other)[sizes[patch] < args.min_patch_cells]] = False
    # Rough patches (blankets, cushions, plants) are not stepped on; they become coarse obstacles
    rough = np.zeros(len(top), dtype=bool)
    rough[np.flatnonzero(other)[~flat]] = True
    rough &= keep
    keep &= ~rough
    top = top.copy()
    top[other] = np.round(flattened / 0.005) * 0.005
    rough_cells = (cx[rough], cy[rough], top[rough])
    return cx[keep], cy[keep], top[keep], rough_cells


def flatten_rugs(cx, cy, top, levels, shape, args) -> np.ndarray:
    """
    Rugs and mats are scanned a few centimeters thick. As walkable slabs they become solid steps with
    sharp edges that catch a walking robot's feet. A flat patch at most args.rug_height above a floor,
    bordered only by that floor or other such patches, is put down onto the floor. A low patch next to
    anything higher, like the first tread of a staircase, stays as it is. A cell can hold surfaces of
    several stories, so each story's surfaces are looked at on their own.
    """
    top = top.copy()
    for level in levels:
        candidate = (top > level + 0.005) & (top <= level + args.rug_height)
        if not candidate.any():
            continue
        # Higher walkable surfaces a step or so above this floor: a stair tread, a landing
        higher = (top > level + args.rug_height) & (top <= level + 2.0 * args.max_step)
        higher_grid = np.zeros(shape, dtype=bool)
        higher_grid[cx[higher], cy[higher]] = True
        rug_grid = np.zeros(shape, dtype=bool)
        rug_grid[cx[candidate], cy[candidate]] = True
        labels, count = ndimage.label(rug_grid)
        cell_label = labels[cx, cy]
        for label in range(1, count + 1):
            patch = labels == label
            border = ndimage.binary_dilation(patch) & ~patch
            if not (border & higher_grid).any():
                top[candidate & (cell_label == label)] = level
    return top


def reachable_from_floors(cx, cy, top, levels, shape, args) -> np.ndarray:
    """
    Surfaces a robot can step onto: the floors, and every flat patch that is within a step's reach
    (horizontally and in height) of a surface already reachable. That follows stairs and landings
    from floor to floor, and leaves out beds and tables, which are higher than a step above the floor.
    """
    on_floor = np.isin(top, levels)
    reachable = on_floor.copy()
    off_floor = np.flatnonzero(~on_floor)
    if len(off_floor) == 0:
        return reachable

    # Off-floor patches: connected cells of equal height
    patch_of_cell = np.full(len(top), -1)
    patches = []
    for height in np.unique(top[off_floor]):
        members = off_floor[top[off_floor] == height]
        mask = np.zeros(shape, dtype=bool)
        mask[cx[members], cy[members]] = True
        labels, count = ndimage.label(mask, structure=np.ones((3, 3)))
        member_labels = labels[cx[members], cy[members]]
        for label in range(1, count + 1):
            patch_of_cell[members[member_labels == label]] = len(patches)
            patches.append((height, members[member_labels == label]))

    # Reachable surfaces on a coarse grid in xy and z, grown one ring of patches per pass
    z_step = 0.05
    z_min = top.min() - args.max_step - z_step
    layers = int(np.ceil((top.max() + args.max_step - z_min) / z_step)) + 2
    reach = np.zeros((shape[0] // 2 + 1, shape[1] // 2 + 1, layers), dtype=bool)
    reach[cx[on_floor] // 2, cy[on_floor] // 2, np.floor((top[on_floor] - z_min) / z_step).astype(np.int64)] = True
    radius = int(np.ceil(args.step_reach / (2 * args.walk_pitch)))
    pending = set(range(len(patches)))
    while pending:
        newly_reached = []
        for index in pending:
            height, members = patches[index]
            x0, x1 = max(cx[members].min() // 2 - radius, 0), cx[members].max() // 2 + radius + 1
            y0, y1 = max(cy[members].min() // 2 - radius, 0), cy[members].max() // 2 + radius + 1
            z0 = max(int((height - args.max_step - z_min) / z_step), 0)
            z1 = int((height + args.max_step - z_min) / z_step) + 1
            if reach[x0:x1, y0:y1, z0:z1].any():
                newly_reached.append(index)
        if not newly_reached:
            break
        for index in newly_reached:
            height, members = patches[index]
            reachable[members] = True
            reach[cx[members] // 2, cy[members] // 2, int((height - z_min) / z_step)] = True
        pending.difference_update(newly_reached)
    return reachable


def greedy_rectangles(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Covers a 2D mask with rectangles (x0, y0, x1, y1), inclusive."""
    mask = mask.copy()
    rectangles = []
    for x in np.flatnonzero(mask.any(axis=1)):
        for y in np.flatnonzero(mask[x]):
            if not mask[x, y]:
                continue
            y1 = y
            while y1 + 1 < mask.shape[1] and mask[x, y1 + 1]:
                y1 += 1
            x1 = x
            while x1 + 1 < mask.shape[0] and mask[x1 + 1, y:y1 + 1].all():
                x1 += 1
            mask[x:x1 + 1, y:y1 + 1] = False
            rectangles.append((x, y, x1, y1))
    return rectangles


def beside_walkable_surface(points, cx, cy, top, origin, pitch, coarse_shape, ratio, args) -> np.ndarray:
    """
    Obstacle points just under or just above a walkable surface nearby: stair risers, wall bases,
    furniture edges. As obstacle columns they would be trip edges or would block a staircase.
    """
    z_step = 0.02
    z_min = top.min() - args.max_slab - z_step
    layers = int(np.ceil((top.max() + args.trip_clearance - z_min) / z_step)) + 2
    near = np.zeros((*coarse_shape, layers), dtype=bool)
    lowest = np.floor((top - args.max_slab - z_min) / z_step).astype(np.int64)
    for layer_offset in range(int(np.ceil((args.max_slab + args.trip_clearance) / z_step)) + 1):
        layer = np.clip(lowest + layer_offset, 0, layers - 1)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                near[np.clip(cx // ratio + dx, 0, coarse_shape[0] - 1), np.clip(cy // ratio + dy, 0, coarse_shape[1] - 1), layer] = True
    px = np.clip(np.floor((points[:, 0] - origin[0]) / (pitch * ratio)).astype(np.int64), 0, coarse_shape[0] - 1)
    py = np.clip(np.floor((points[:, 1] - origin[1]) / (pitch * ratio)).astype(np.int64), 0, coarse_shape[1] - 1)
    pz = np.floor((points[:, 2] - z_min) / z_step).astype(np.int64)
    inside = (pz >= 0) & (pz < layers)
    result = np.zeros(len(points), dtype=bool)
    result[inside] = near[px[inside], py[inside], pz[inside]]
    return result


def load_geometry(glb: Path) -> trimesh.Trimesh:
    """
    The scene's triangles in world coordinates, without materials. Decoding the textures and
    merging them into one atlas takes tens of GB for an HM3D scene; geometry alone is a few hundred MB.
    """
    scene = trimesh.load(str(glb), force="scene", skip_materials=True)
    parts = []
    for node in scene.graph.nodes_geometry:
        transform, geometry_name = scene.graph[node]
        geometry = scene.geometry[geometry_name]
        parts.append(trimesh.Trimesh(vertices=trimesh.transform_points(geometry.vertices, transform), faces=geometry.faces, process=False))
    return trimesh.util.concatenate(parts)


GLTF_COMPONENT_TYPES = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
GLTF_TYPE_WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def read_accessor(gltf: GLTF2, blob: bytes, index: int) -> np.ndarray:
    accessor = gltf.accessors[index]
    view = gltf.bufferViews[accessor.bufferView]
    dtype = np.dtype(GLTF_COMPONENT_TYPES[accessor.componentType])
    width = GLTF_TYPE_WIDTHS[accessor.type]
    if view.byteStride and view.byteStride != width * dtype.itemsize:
        raise ValueError(f"Interleaved accessor {index} is not supported")
    start = (view.byteOffset or 0) + (accessor.byteOffset or 0)
    # A copy, since a view into the buffer would stop it from growing
    return np.frombuffer(blob, dtype, accessor.count * width, start).reshape(accessor.count, width).copy()


def append_array(gltf: GLTF2, blob: bytearray, array: np.ndarray, accessor_type: str, target: int | None = None,
                 bounds: bool = False) -> int:
    """Appends an array to the GLB buffer as a new accessor and returns its index."""
    component_type = {np.dtype(np.float32): 5126, np.dtype(np.uint16): 5123, np.dtype(np.uint32): 5125}[array.dtype]
    blob.extend(b"\0" * (-len(blob) % 4))
    gltf.bufferViews.append(BufferView(buffer=0, byteOffset=len(blob), byteLength=array.nbytes, target=target))
    blob.extend(array.tobytes())
    rows = array.reshape(len(array), -1)
    gltf.accessors.append(Accessor(bufferView=len(gltf.bufferViews) - 1, componentType=component_type, count=len(array), type=accessor_type,
                                   min=rows.min(axis=0).tolist() if bounds else None, max=rows.max(axis=0).tolist() if bounds else None))
    return len(gltf.accessors) - 1


def index_array(faces: np.ndarray, vertex_count: int) -> np.ndarray:
    return faces.astype(np.uint16 if vertex_count <= 65535 else np.uint32).ravel()


def split_polygon(points: np.ndarray, uvs: np.ndarray, distance: np.ndarray):
    """Splits a convex polygon where the signed distance crosses zero, into the parts at or above and at or below zero."""
    above, below = ([], []), ([], [])
    for i in range(len(points)):
        j = (i + 1) % len(points)
        if distance[i] >= 0.0:
            above[0].append(points[i])
            above[1].append(uvs[i])
        if distance[i] <= 0.0:
            below[0].append(points[i])
            below[1].append(uvs[i])
        if distance[i] * distance[j] < 0.0:
            t = distance[i] / (distance[i] - distance[j])
            for part in (above, below):
                part[0].append(points[i] + t * (points[j] - points[i]))
                part[1].append(uvs[i] + t * (uvs[j] - uvs[i]))
    return tuple((np.array(part[0]), np.array(part[1])) if len(part[0]) >= 3 else None for part in (above, below))


def polygon_area(points: np.ndarray) -> float:
    return 0.5 * float(np.linalg.norm(np.cross(points[1:-1] - points[0], points[2:] - points[0]).sum(axis=0))) if len(points) >= 3 else 0.0


def clip_out_box(points: np.ndarray, uvs: np.ndarray, low: np.ndarray, high: np.ndarray):
    """
    The parts of a convex polygon outside an axis-aligned box, as convex polygons, or None when it
    does not reach into the box. Each face plane of the box in turn splits off what lies beyond it.
    """
    kept = []
    remaining = (points, uvs)
    for axis in range(3):
        for bound, sign in ((low[axis], -1.0), (high[axis], 1.0)):
            outside, inside = split_polygon(*remaining, sign * (remaining[0][:, axis] - bound))
            if inside is None or polygon_area(inside[0]) < 1e-9:
                return None
            if outside is not None and polygon_area(outside[0]) >= 1e-9:
                kept.append(outside)
            remaining = inside
    return kept


def cut_primitive(gltf: GLTF2, blob: bytearray, primitive, cuts) -> int:
    """
    Removes what lies inside the cuts from a primitive, splitting the triangles that cross a cut's
    faces exactly on them, so the opening has straight edges. Returns how many triangles changed.
    """
    positions = read_accessor(gltf, blob, primitive.attributes.POSITION).astype(np.float64)
    uvs = read_accessor(gltf, blob, primitive.attributes.TEXCOORD_0).astype(np.float64)
    faces = read_accessor(gltf, blob, primitive.indices).reshape(-1, 3).astype(np.int64)
    triangles = positions[faces]
    lows, highs = triangles.min(axis=1), triangles.max(axis=1)
    candidates = np.zeros(len(faces), dtype=bool)
    for low, high in cuts:
        candidates |= np.all((highs > low) & (lows < high), axis=1)
    if not candidates.any():
        return 0

    new_points, new_uvs, keep = [], [], np.ones(len(faces), dtype=bool)
    changed = 0
    for face_index in np.flatnonzero(candidates):
        polygons = [(positions[faces[face_index]], uvs[faces[face_index]])]
        touched = False
        for low, high in cuts:
            pieces = []
            for polygon in polygons:
                outside = clip_out_box(*polygon, low, high)
                if outside is None:
                    pieces.append(polygon)
                else:
                    pieces.extend(outside)
                    touched = True
            polygons = pieces
        if not touched:
            continue
        keep[face_index] = False
        changed += 1
        for points, polygon_uvs in polygons:
            for corner in range(1, len(points) - 1):
                fan = [0, corner, corner + 1]
                if polygon_area(points[fan]) >= 1e-9:
                    new_points.append(points[fan])
                    new_uvs.append(polygon_uvs[fan])
    if not changed:
        return 0
    added = len(new_points)
    if added:
        positions = np.vstack([positions, np.concatenate(new_points)])
        uvs = np.vstack([uvs, np.concatenate(new_uvs)])
    first_new = len(positions) - 3 * added
    faces = np.vstack([faces[keep], np.arange(first_new, len(positions)).reshape(-1, 3)])
    if len(faces) == 0:
        # The whole piece was inside a cut. One degenerate triangle keeps the primitive valid and draws nothing.
        faces = np.zeros((1, 3), dtype=np.int64)
    primitive.attributes.POSITION = append_array(gltf, blob, positions.astype(np.float32), "VEC3", 34962, bounds=True)
    primitive.attributes.TEXCOORD_0 = append_array(gltf, blob, uvs.astype(np.float32), "VEC2", 34962)
    primitive.indices = append_array(gltf, blob, index_array(faces, len(positions)), "SCALAR", 34963)
    return changed


def smooth_normals(positions: np.ndarray, faces: np.ndarray) -> np.ndarray:
    triangles = positions[faces]
    face_normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals = np.zeros_like(positions)
    for corner in range(3):
        np.add.at(normals, faces[:, corner], face_normals)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    return np.where(lengths > 0.0, normals / np.maximum(lengths, 1e-12), [0.0, 0.0, 1.0]).astype(np.float32)


# The outside of the building, which the scan never saw: gray blocks in rows, so a simulated camera
# sees a plain masonry wall, not a bright white panel that a door detector takes for a door.
EXTERIOR_TILE_METERS = 1.0
# Blocks whose size divides the tile, with an even number of rows, so the texture repeats seamlessly
EXTERIOR_BLOCK = (0.5, 0.25)


def exterior_texture_png(pixels_per_meter: int = 512) -> bytes:
    import io
    from PIL import Image
    rng = np.random.default_rng(7)
    size = int(EXTERIOR_TILE_METERS * pixels_per_meter)
    meters = (np.arange(size) + 0.5) / pixels_per_meter
    row = np.floor(meters / EXTERIOR_BLOCK[1]).astype(int)
    u = meters[None, :] + 0.5 * EXTERIOR_BLOCK[0] * (row[:, None] % 2)
    column = np.floor(u / EXTERIOR_BLOCK[0]).astype(int) % int(round(EXTERIOR_TILE_METERS / EXTERIOR_BLOCK[0]))
    shade = rng.normal(0.0, 0.035, (row.max() + 1, column.max() + 1))[row[:, None], column]
    grain = ndimage.gaussian_filter(rng.normal(0.0, 1.0, (size, size)), 1.5, mode="wrap")
    value = 0.50 + shade + 0.03 * grain / grain.std()
    mortar = 0.012
    in_mortar = (np.minimum(meters % EXTERIOR_BLOCK[1], EXTERIOR_BLOCK[1] - meters % EXTERIOR_BLOCK[1])[:, None] < 0.5 * mortar) \
                | (np.minimum(u % EXTERIOR_BLOCK[0], EXTERIOR_BLOCK[0] - u % EXTERIOR_BLOCK[0]) < 0.5 * mortar)
    value = np.where(in_mortar, 0.33 + 0.01 * grain / grain.std(), value)
    gray = np.clip(value * 255.0, 0, 255).astype(np.uint8)
    # A little warmth, like concrete
    image = np.stack([gray, gray, np.clip(gray.astype(int) - 6, 0, 255).astype(np.uint8)], axis=-1)
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="PNG")
    return buffer.getvalue()


def add_exterior_material(gltf: GLTF2, blob: bytearray) -> int:
    png = exterior_texture_png()
    blob.extend(b"\0" * (-len(blob) % 4))
    gltf.bufferViews.append(BufferView(buffer=0, byteOffset=len(blob), byteLength=len(png)))
    blob.extend(png)
    gltf.images.append(Image(bufferView=len(gltf.bufferViews) - 1, mimeType="image/png"))
    gltf.samplers.append(Sampler(magFilter=9729, minFilter=9987, wrapS=10497, wrapT=10497))
    gltf.textures.append(Texture(sampler=len(gltf.samplers) - 1, source=len(gltf.images) - 1))
    gltf.materials.append(Material(name="hm3d_exterior", doubleSided=False,
                                   pbrMetallicRoughness=PbrMetallicRoughness(baseColorFactor=[1.0, 1.0, 1.0, 1.0], metallicFactor=0.0,
                                                                             roughnessFactor=1.0,
                                                                             baseColorTexture=TextureInfo(index=len(gltf.textures) - 1))))
    return len(gltf.materials) - 1


def write_visual(source: Path, destination: Path, offset: np.ndarray, cuts: list[tuple[np.ndarray, np.ndarray]] = ()) -> None:
    """
    Copies the visual GLB for RDX:

    * Cuts out what lies inside each cut, an axis-aligned (min, max) box in the GLB's coordinates,
      which opens a doorway in the wall.
    * Adds smooth vertex normals. HM3D meshes have only positions and texture coordinates, and the
      RDX PBR shader lights a surface without normals as plain white.
    * Gives every surface a back side. The scan only has the inside of each wall, facing the rooms.
      The back of each triangle is drawn with a gray block texture, so from outside the building
      looks like a masonry wall, and simulated cameras do not see through it. ``offset`` moves the
      GLB's coordinates to the object frame, so the block rows sit level with its floor.
    """
    gltf = GLTF2().load(str(source))
    if any(node.matrix or node.translation or node.rotation or node.scale for node in gltf.nodes):
        raise SystemExit("The visual needs a GLB whose nodes have no transforms")
    blob = bytearray(gltf.binary_blob())
    if cuts:
        changed = sum(cut_primitive(gltf, blob, primitive, cuts) for mesh in gltf.meshes for primitive in mesh.primitives
                      if primitive.indices is not None and primitive.attributes.TEXCOORD_0 is not None)
        print(f"  cut {changed} visual triangles along the doorway")

    exterior_material = add_exterior_material(gltf, blob)
    for mesh in gltf.meshes:
        for primitive in list(mesh.primitives):
            positions = read_accessor(gltf, blob, primitive.attributes.POSITION).astype(np.float64)
            if primitive.indices is not None:
                faces = read_accessor(gltf, blob, primitive.indices).reshape(-1, 3).astype(np.int64)
            else:
                faces = np.arange(len(positions)).reshape(-1, 3)
            normals = smooth_normals(positions, faces)
            if primitive.attributes.NORMAL is None:
                primitive.attributes.NORMAL = append_array(gltf, blob, normals, "VEC3", 34962, bounds=True)
            gltf.materials[primitive.material].doubleSided = False

            # Wall-aligned texture coordinates: along the wall from x + y, up from z
            world = positions + offset
            exterior_uvs = np.stack([world[:, 0] + world[:, 1], -world[:, 2]], axis=1) / EXTERIOR_TILE_METERS
            mesh.primitives.append(Primitive(
                attributes=Attributes(POSITION=primitive.attributes.POSITION,
                                      NORMAL=append_array(gltf, blob, -normals, "VEC3", 34962, bounds=True),
                                      TEXCOORD_0=append_array(gltf, blob, exterior_uvs.astype(np.float32), "VEC2", 34962)),
                indices=append_array(gltf, blob, index_array(faces[:, ::-1], len(positions)), "SCALAR", 34963),
                material=exterior_material, mode=4))
    gltf.buffers[0].byteLength = len(blob)
    gltf.set_binary_blob(bytes(blob))
    gltf.save_binary(str(destination))


# RDXArticulatedDoorObject, from DoorModelParameters: the frame origin is on the hinge jamb, the opening
# runs along +y, and the wall is the frame's yz plane. These span both jambs and the panel.
DOOR_OPENING_MIN_Y = 0.002 - 0.10
DOOR_OPENING_MAX_Y = 0.002 + 0.924 + 0.10
DOOR_OPENING_HEIGHT = 0.02 + 2.033
DOOR_FRAME_PILLAR_HEIGHT = 2.159
# The outside of its visual frame, door/doorFrame/DoorFrame.glb: the trim around the opening
DOOR_FRAME_OUTER_MIN_Y = -0.147
DOOR_FRAME_OUTER_MAX_Y = 1.075
DOOR_FRAME_OUTER_TOP = 2.14
DOOR_FRAME_OUTSIDE_X = -0.03
# The scan is cut this far inside the trim, which hides the cut edge
DOOR_FRAME_TRIM_OVERLAP = 0.02


def door_cut(door: tuple[float, float, float], depth: float, min_y: float = DOOR_OPENING_MIN_Y - 0.01,
             max_y: float = DOOR_OPENING_MAX_Y + 0.01, top: float = DOOR_FRAME_PILLAR_HEIGHT + 0.05) -> tuple[np.ndarray, np.ndarray]:
    """The doorway through the wall for a door at (x, y, yaw degrees) in the object frame, as an axis-aligned box from the floor up."""
    x, y, yaw_degrees = door
    if abs(yaw_degrees / 90.0 - round(yaw_degrees / 90.0)) > 1e-6:
        raise SystemExit("--door yaw must be a multiple of 90 degrees, so the doorway lines up with the collision boxes")
    yaw = np.radians(yaw_degrees)
    corners = np.array([[dx, dy] for dx in (-depth, depth) for dy in (min_y, max_y)])
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    corners = corners @ rotation.T + [x, y]
    return np.array([*corners.min(axis=0), 0.0]), np.array([*corners.max(axis=0), top])


def snap_door_to_wall(mesh: trimesh.Trimesh, offset: np.ndarray, door: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Moves the door through the wall so the outside of its frame sits just proud of the scanned wall,
    which only has its inside surface. Then the frame trim covers the cut edge and does not stand
    back inside the room.
    """
    x, y, yaw_degrees = door
    yaw = np.radians(yaw_degrees)
    inward = np.array([np.cos(yaw), np.sin(yaw)])
    along = np.array([-np.sin(yaw), np.cos(yaw)])
    centers = mesh.triangles_center + offset
    relative = centers[:, :2] - [x, y]
    depth = relative @ inward
    lateral = relative @ along
    wall = ((np.abs(depth) < 0.5) & (lateral > DOOR_FRAME_OUTER_MIN_Y - 0.6) & (lateral < DOOR_FRAME_OUTER_MAX_Y + 0.6)
            & (centers[:, 2] > 0.3) & (centers[:, 2] < 2.0) & (mesh.face_normals[:, :2] @ inward > 0.7))
    area = mesh.area_faces[wall]
    if area.sum() < 0.3:
        print("  no wall found at the door to line it up with, keeping its pose")
        return door
    order = np.argsort(depth[wall])
    wall_depth = depth[wall][order][np.searchsorted(np.cumsum(area[order]), 0.5 * area.sum())]
    shift = wall_depth - 0.01 - DOOR_FRAME_OUTSIDE_X
    snapped = (round(float(x + shift * inward[0]), 4), round(float(y + shift * inward[1]), 4), yaw_degrees)
    print(f"  wall at {wall_depth:+.3f} m from the door, moved the door {shift:+.3f} m to ({snapped[0]}, {snapped[1]})")
    return snapped


def subtract_box(box: np.ndarray, low: np.ndarray, high: np.ndarray) -> list[np.ndarray]:
    """A (sizeX sizeY sizeZ x y z) box minus an axis-aligned (low, high) box, as up to six boxes."""
    box_low, box_high = box[3:] - 0.5 * box[:3], box[3:] + 0.5 * box[:3]
    if np.any(box_high <= low) or np.any(box_low >= high):
        return [box]
    pieces = []
    remaining_low, remaining_high = box_low.copy(), box_high.copy()
    for axis in range(3):
        if remaining_low[axis] < low[axis]:
            piece_high = remaining_high.copy()
            piece_high[axis] = low[axis]
            pieces.append((remaining_low.copy(), piece_high))
            remaining_low[axis] = low[axis]
        if remaining_high[axis] > high[axis]:
            piece_low = remaining_low.copy()
            piece_low[axis] = high[axis]
            pieces.append((piece_low, remaining_high.copy()))
            remaining_high[axis] = high[axis]
    return [np.concatenate([piece_high - piece_low, 0.5 * (piece_low + piece_high)]) for piece_low, piece_high in pieces
            if np.all(piece_high - piece_low > 1e-4)]


def ground_floor_entrances(boxes: np.ndarray, walkable_count: int, resolution: float = 0.1) -> list[dict]:
    """
    Openings where the ground floor meets the outside through a gap in the walls, most door-like
    first. Each has its center and width, and the yaw of the direction out of the building, in the
    object frame. A glass door or window down to the floor is open in the scan, so it counts too.
    """
    low = boxes[:, 3:5].min(axis=0) - boxes[:, :2].max() - 1.0
    high = boxes[:, 3:5].max(axis=0) + boxes[:, :2].max() + 1.0
    shape = tuple(np.ceil((high - low) / resolution).astype(int) + 1)
    floor = np.zeros(shape, dtype=bool)
    wall = np.zeros(shape, dtype=bool)
    for index, (size_x, size_y, size_z, x, y, z) in enumerate(boxes):
        cells = (slice(int(round((x - 0.5 * size_x - low[0]) / resolution)), int(round((x + 0.5 * size_x - low[0]) / resolution))),
                 slice(int(round((y - 0.5 * size_y - low[1]) / resolution)), int(round((y + 0.5 * size_y - low[1]) / resolution))))
        if index < walkable_count and abs(z + 0.5 * size_z) < 0.01:
            floor[cells] = True
        elif index >= walkable_count and abs(z - 0.5 * size_z) < 0.01 and size_z > 1.0:
            wall[cells] = True
    labels, _ = ndimage.label(~floor & ~wall)
    border = np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]]))
    outside = np.isin(labels, border[border > 0])
    threshold = floor & ~wall & ndimage.binary_dilation(outside)
    labels, count = ndimage.label(threshold, structure=np.ones((3, 3)))
    entrances = []
    for label in range(1, count + 1):
        ix, iy = np.nonzero(labels == label)
        width = (max(np.ptp(ix), np.ptp(iy)) + 1) * resolution
        if width < 0.7:
            continue
        # Out of the building is toward the outside cells next to the threshold
        near = ndimage.binary_dilation(labels == label, iterations=3) & outside
        ox, oy = np.nonzero(near)
        yaw = np.arctan2(oy.mean() - iy.mean(), ox.mean() - ix.mean())
        entrances.append({"x": round(float(low[0] + (ix.mean() + 0.5) * resolution), 3),
                          "y": round(float(low[1] + (iy.mean() + 0.5) * resolution), 3),
                          "width": round(float(width), 2),
                          "outwardYaw": round(float(yaw), 3)})
    # A single door is about a meter wide
    return sorted(entrances, key=lambda entrance: abs(entrance["width"] - 1.0))


def convert(glb: Path, args: argparse.Namespace) -> None:
    start = time.time()
    scene_id = glb.parent.name if glb.parent.name.endswith(glb.stem) else glb.stem
    mesh = load_geometry(glb)
    print(f"{scene_id}: {len(mesh.faces)} faces")
    rng = np.random.default_rng(0)

    normal_z = mesh.face_normals[:, 2]
    walkable_faces = np.flatnonzero(normal_z > WALKABLE_NORMAL_Z)
    # Ceilings are covered by the slab of the floor above, so only near-vertical faces are obstacles
    obstacle_faces = np.flatnonzero(np.abs(normal_z) <= WALKABLE_NORMAL_Z)

    pitch = args.walk_pitch
    bounds = mesh.bounds
    origin = bounds[0] - 2.0 * args.obstacle_pitch
    shape = tuple(np.ceil((bounds[1] - origin) / pitch).astype(int)[:2] + 4)

    # Walkable surfaces
    points = sample_faces(mesh, walkable_faces, pitch / 3.0, rng)
    cx, cy, top, count = walkable_surfaces(points, origin, pitch, shape)
    keep = count >= 3
    cx, cy, top = cx[keep], cy[keep], top[keep]
    levels = floor_levels(top, np.ones_like(top), args.min_floor_area, pitch * pitch, args.min_story_height)
    for level in levels:
        top[np.abs(top - level) < args.floor_snap] = level
    snapped = np.isin(top, levels)
    top[~snapped] = np.round(top[~snapped] / args.step_quantum) * args.step_quantum
    cx, cy, top, (rough_x, rough_y, rough_top) = flatten_patches(cx, cy, top, snapped, shape, args)
    top = flatten_rugs(cx, cy, top, levels, shape, args)
    # Furniture tops are not walked on: they become obstacles, so only stairs and landings stay walkable
    reachable = reachable_from_floors(cx, cy, top, levels, shape, args)
    rough_x, rough_y, rough_top = (np.append(rough_x, cx[~reachable]), np.append(rough_y, cy[~reachable]),
                                   np.append(rough_top, top[~reachable]))
    cx, cy, top = cx[reachable], cy[reachable], top[reachable]

    # Obstacles: walls and furniture, sampled from their near-vertical faces and rough tops
    coarse = args.obstacle_pitch
    ratio = int(round(coarse / pitch))
    coarse_shape = (int(np.ceil(shape[0] / ratio)), int(np.ceil(shape[1] / ratio)))
    obstacle_points = sample_faces(mesh, obstacle_faces, coarse / 3.0, rng)
    rough_points = np.stack([origin[0] + (rough_x + 0.5) * pitch, origin[1] + (rough_y + 0.5) * pitch, rough_top], axis=1)
    obstacle_points = np.vstack([obstacle_points, rough_points])
    obstacle_points = obstacle_points[~beside_walkable_surface(obstacle_points, cx, cy, top, origin, pitch, coarse_shape, ratio, args)]
    obstacle_x = np.floor((obstacle_points[:, 0] - origin[0]) / coarse).astype(np.int64)
    obstacle_y = np.floor((obstacle_points[:, 1] - origin[1]) / coarse).astype(np.int64)

    obstacle_boxes = []
    story_tops = levels[1:] + [bounds[1][2] + args.max_slab]
    for level, story_top in zip(levels, story_tops):
        # Each story's obstacles are columns from its floor up to the tallest obstacle in the cell
        ceiling = story_top - args.max_slab
        in_story = (obstacle_points[:, 2] > level + args.trip_clearance) & (obstacle_points[:, 2] < ceiling)
        # Only what the robot can bump into makes a cell an obstacle. A cell with scan points only
        # overhead (a door header, a beam, a lowered ceiling) stays open: a column from the floor up
        # to it would be a wall the scan does not show.
        within_reach = in_story & (obstacle_points[:, 2] < level + args.obstacle_reach_height)
        hits = np.zeros(coarse_shape, dtype=np.int64)
        np.add.at(hits, (obstacle_x[within_reach], obstacle_y[within_reach]), 1)
        # The floor is still filled under overhead-only cells, as it is under every column
        hits_any_height = np.zeros(coarse_shape, dtype=np.int64)
        np.add.at(hits_any_height, (obstacle_x[in_story], obstacle_y[in_story]), 1)
        overhead_only = (hits_any_height >= args.min_obstacle_points) & (hits < args.min_obstacle_points)
        tallest = np.full(coarse_shape, -np.inf)
        np.maximum.at(tallest, (obstacle_x[in_story], obstacle_y[in_story]), obstacle_points[in_story, 2])
        occupied = hits >= args.min_obstacle_points
        labels, label_count = ndimage.label(occupied)
        if label_count:
            sizes = np.bincount(labels.ravel(), minlength=label_count + 1)
            occupied &= sizes[labels] >= args.min_obstacle_cells
        # Walls reach the ceiling, so tall obstacles share one height and merge into few boxes.
        # Others round down to a coarse step.
        height = np.floor((tallest - level) / args.obstacle_height_step) * args.obstacle_height_step
        height[tallest - level >= args.full_height_above] = ceiling - level
        # A column never reaches a stair tread or landing above it in the same cell
        in_story_surface = (top > level + args.trip_clearance) & (top < ceiling)
        lowest_surface = np.full(coarse_shape, np.inf)
        np.minimum.at(lowest_surface, (cx[in_story_surface] // ratio, cy[in_story_surface] // ratio), top[in_story_surface])
        under_surface = np.isfinite(lowest_surface)
        height[under_surface] = np.minimum(height[under_surface],
                                           np.floor((lowest_surface[under_surface] - args.max_slab - level) / args.obstacle_height_step)
                                           * args.obstacle_height_step)
        occupied &= height > args.trip_clearance
        for column_height in np.unique(height[occupied]):
            for x0, y0, x1, y1 in greedy_rectangles(occupied & (height == column_height)):
                obstacle_boxes.append(((x1 - x0 + 1) * coarse, (y1 - y0 + 1) * coarse, column_height,
                                       origin[0] + 0.5 * (x0 + x1 + 1) * coarse, origin[1] + 0.5 * (y0 + y1 + 1) * coarse,
                                       level + 0.5 * column_height))

        # Furniture tops and clutter under or on a column are covered by it; only stairs and landings stay
        column_top = np.where(occupied, level + height, -np.inf)
        covered = in_story_surface & (column_top[cx // ratio, cy // ratio] >= top - 0.05)
        cx, cy, top = cx[~covered], cy[~covered], top[~covered]

        # Fill the floor under walls, furniture and anything standing on it, and fill scan holes, but
        # leave openings where a stair or a lower surface is visible less than a story below
        floor_cells = top == level
        at_level = np.zeros(shape, dtype=bool)
        at_level[cx[floor_cells], cy[floor_cells]] = True
        standing = np.zeros(shape, dtype=bool)
        above = (top > level + 0.03) & (top < level + args.fill_under_height)
        standing[cx[above], cy[above]] = True
        rough_above = (rough_top > level) & (rough_top < level + args.fill_under_height)
        standing[rough_x[rough_above], rough_y[rough_above]] = True
        standing |= np.kron(occupied | overhead_only, np.ones((ratio, ratio), dtype=bool))[:shape[0], :shape[1]]
        open_below = np.zeros(shape, dtype=bool)
        below = (top < level - 0.03) & (top > level - args.min_story_height)
        open_below[cx[below], cy[below]] = True
        open_below = ndimage.binary_dilation(open_below)
        floor = (at_level | standing) & ~open_below | at_level
        holes = ndimage.binary_fill_holes(floor) & ~floor & ~open_below
        hole_labels, hole_count = ndimage.label(holes)
        if hole_count:
            hole_sizes = np.bincount(hole_labels.ravel(), minlength=hole_count + 1)
            floor |= holes & (hole_sizes[hole_labels] * pitch * pitch <= args.max_hole_area)
        fx, fy = np.nonzero(floor & ~at_level)
        cx, cy, top = np.append(cx, fx), np.append(cy, fy), np.append(top, np.full(len(fx), level))

    # Each surface becomes a slab down to the next surface below, so stairs are solid steps
    order = np.lexsort((top, cx * shape[1] + cy))
    cx, cy, top = cx[order], cy[order], top[order]
    same_column = np.zeros(len(top), dtype=bool)
    same_column[1:] = (cx[1:] == cx[:-1]) & (cy[1:] == cy[:-1])
    below = np.full(len(top), -np.inf)
    below[1:][same_column[1:]] = top[:-1][same_column[1:]]
    thickness = np.clip(top - below - 0.02, args.min_slab, args.max_slab)
    # Coarse thickness steps so neighboring cells of one surface share a box
    thickness = np.maximum(np.floor(thickness / 0.05) * 0.05, args.min_slab)

    walk_boxes = []
    keys = np.stack([np.round(top / 0.005), np.round(thickness / 0.005)], axis=1).astype(np.int64)
    unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
    for key_index, (top_key, thickness_key) in enumerate(unique_keys):
        members = inverse.ravel() == key_index
        mask = np.zeros(shape, dtype=bool)
        mask[cx[members], cy[members]] = True
        height = top_key * 0.005
        slab = thickness_key * 0.005
        for x0, y0, x1, y1 in greedy_rectangles(mask):
            walk_boxes.append(((x1 - x0 + 1) * pitch, (y1 - y0 + 1) * pitch, slab,
                               origin[0] + 0.5 * (x0 + x1 + 1) * pitch, origin[1] + 0.5 * (y0 + y1 + 1) * pitch, height - 0.5 * slab))

    # Object frame: centered on the footprint, z = 0 at the lowest floor
    ground = levels[0] if levels else bounds[0][2]
    offset = np.array([-0.5 * (bounds[0][0] + bounds[1][0]), -0.5 * (bounds[0][1] + bounds[1][1]), -ground])
    walk = np.array(walk_boxes)
    obstacles = np.array(obstacle_boxes)
    walk[:, 3:] += offset
    obstacles[:, 3:] += offset

    # Scans rarely have an open exterior door. Cut a doorway for an RDXArticulatedDoorObject through
    # the wall: out of the visual and the collision, keeping and completing the floor.
    visual_cuts = []
    doors = []
    entrances = None

    def cut_doorway(low, high):
        """Removes the walls in the (low, high) box from the collision, keeping and completing the floor under it."""
        nonlocal walk, obstacles
        above_floor = low.copy()
        above_floor[2] = 0.03
        through_floor = low.copy()
        through_floor[2] = -args.max_slab - 1.0
        walk = np.array([piece for box in walk for piece in subtract_box(box, above_floor, high)])
        obstacles = np.array([piece for box in obstacles for piece in subtract_box(box, through_floor, high)])
        walk = np.vstack([walk, [[high[0] - low[0], high[1] - low[1], args.max_slab,
                                  0.5 * (low[0] + high[0]), 0.5 * (low[1] + high[1]), -0.5 * args.max_slab]]])

    if args.door:
        args.door = snap_door_to_wall(mesh, offset, args.door)
        low, high = door_cut(args.door, args.door_wall_depth)
        cut_doorway(low, high)
        # The visual opening is the outside of the door frame, inside its trim
        visual_low, visual_high = door_cut(args.door, args.door_wall_depth, DOOR_FRAME_OUTER_MIN_Y + DOOR_FRAME_TRIM_OVERLAP,
                                           DOOR_FRAME_OUTER_MAX_Y - DOOR_FRAME_TRIM_OVERLAP, DOOR_FRAME_OUTER_TOP - DOOR_FRAME_TRIM_OVERLAP)
        visual_low[2] = 0.03
        visual_cuts.append((visual_low - offset, visual_high - offset))
        x, y, yaw_degrees = args.door
        yaw = float(np.radians(yaw_degrees))
        doors.append({"x": x, "y": y, "z": 0.0, "yaw": round(yaw, 4)})
        center = 0.5 * (DOOR_OPENING_MIN_Y + DOOR_OPENING_MAX_Y)
        entrances = [{"x": round(x - np.sin(yaw) * center, 3), "y": round(y + np.cos(yaw) * center, 3),
                      "width": round(DOOR_OPENING_MAX_Y - DOOR_OPENING_MIN_Y, 2),
                      # The door's +x points into the building
                      "outwardYaw": round(float(np.arctan2(-np.sin(yaw), -np.cos(yaw))), 3)}]
    # Doorways without a door, the size of the door's, where the scan has a wall or a closed door
    # across a passage. Cut out of the collision and the visual alike, so the robot sees what it walks.
    for x, y, yaw_degrees in args.opening or []:
        yaw = np.radians(yaw_degrees)
        center = 0.5 * (DOOR_OPENING_MIN_Y + DOOR_OPENING_MAX_Y)
        # door_cut takes the hinge jamb; the opening's center is half its width along +y from there
        hinge = (x + np.sin(yaw) * center, y - np.cos(yaw) * center, yaw_degrees)
        low, high = door_cut(hinge, args.door_wall_depth)
        cut_doorway(low, high)
        visual_low = low.copy()
        visual_low[2] = 0.03
        visual_cuts.append((visual_low - offset, high - offset))
        print(f"  cut an opening {high[0] - low[0]:.2f} x {high[1] - low[1]:.2f} m, {high[2]:.2f} m tall, at ({x}, {y})")
    boxes = np.vstack([walk, obstacles])

    out = HM3D_RESOURCES / scene_id
    out.mkdir(parents=True, exist_ok=True)
    write_visual(glb, out / f"{scene_id}.glb", offset, visual_cuts)
    np.savetxt(out / f"{scene_id}_collision_boxes.txt", boxes, fmt="%.4f",
               header=f"{scene_id}: sizeX sizeY sizeZ x y z in the object frame. {len(walk)} walkable, {len(obstacles)} obstacle.")
    metadata = {
        "scene": scene_id,
        "source": str(glb),
        "visualOffset": [float(v) for v in offset],
        "floorHeights": [round(float(level + offset[2]), 3) for level in levels],
        "boundsMin": [round(float(v), 3) for v in bounds[0] + offset],
        "boundsMax": [round(float(v), 3) for v in bounds[1] + offset],
        "walkableBoxes": len(walk),
        "obstacleBoxes": len(obstacles),
        "doors": doors,
        "entrances": entrances if entrances is not None else ground_floor_entrances(boxes, len(walk)),
    }
    (out / f"{scene_id}_rdx.json").write_text(json.dumps(metadata, indent=2) + "\n")
    index = HM3D_RESOURCES / "scenes.txt"
    scenes = set(index.read_text().split()) if index.exists() else set()
    scenes.add(scene_id)
    index.write_text("\n".join(sorted(scenes)) + "\n")
    print(f"  floors at {metadata['floorHeights']} m")
    print(f"  {len(walk)} walkable + {len(obstacles)} obstacle boxes in {time.time() - start:.0f} s -> {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("glb", type=Path, nargs="+", help="HM3D scene GLB(s), e.g. 00337-CFVBbU9Rsyb/CFVBbU9Rsyb.glb")
    parser.add_argument("--walk-pitch", type=float, default=0.05, help="Cell size of walkable surfaces (m)")
    parser.add_argument("--obstacle-pitch", type=float, default=0.10, help="Voxel size of walls and furniture (m)")
    parser.add_argument("--step-quantum", type=float, default=0.02, help="Height rounding of surfaces that are not floors (m)")
    parser.add_argument("--floor-snap", type=float, default=0.04, help="Surfaces this close to a floor height snap to it (m)")
    parser.add_argument("--min-floor-area", type=float, default=4.0, help="Flat area needed to count as a floor (m^2)")
    parser.add_argument("--min-story-height", type=float, default=1.8, help="Floors of two stories are at least this far apart (m)")
    parser.add_argument("--fill-under-height", type=float, default=1.8,
                        help="A floor is filled under anything standing on it up to this height (m)")
    parser.add_argument("--rug-height", type=float, default=0.10,
                        help="Flat patches this low above a floor, bordered only by it, are rugs: they are put down onto the floor (m)")
    parser.add_argument("--min-slab", type=float, default=0.04, help="Minimum thickness of a walkable slab (m)")
    parser.add_argument("--max-slab", type=float, default=0.30, help="Maximum thickness of a walkable slab (m)")
    parser.add_argument("--trip-clearance", type=float, default=0.12,
                        help="Obstacle voxels whose top is within this height above a walkable surface are dropped (m)")
    parser.add_argument("--max-step", type=float, default=0.25, help="Highest rise between walkable surfaces the robot can step (m)")
    parser.add_argument("--step-reach", type=float, default=0.35, help="Farthest gap between walkable surfaces the robot can step across (m)")
    parser.add_argument("--min-patch-cells", type=int, default=16,
                        help="Off-floor surface patches with fewer cells are scan noise, too small to step on")
    parser.add_argument("--flat-patch-range", type=float, default=0.06,
                        help="Off-floor patches whose heights span at most this much are flattened (m)")
    parser.add_argument("--obstacle-height-step", type=float, default=0.25, help="Obstacle column heights are rounded down to this (m)")
    parser.add_argument("--obstacle-reach-height", type=float, default=1.9,
                        help="Only scan surfaces below this height above a floor make an obstacle; surfaces only above it "
                             "(door headers, beams, lowered ceilings) are overhead, not walls (m)")
    parser.add_argument("--full-height-above", type=float, default=1.5,
                        help="Obstacles at least this tall (walls) reach the ceiling (m)")
    parser.add_argument("--min-obstacle-points", type=int, default=5, help="Fewer scan points in an obstacle cell is noise")
    parser.add_argument("--min-obstacle-cells", type=int, default=4, help="Smaller obstacle clusters are scan noise")
    parser.add_argument("--max-hole-area", type=float, default=1.5,
                        help="Enclosed floor holes up to this area are filled; larger ones are openings (m^2)")
    parser.add_argument("--door", type=float, nargs=3, metavar=("X", "Y", "YAW_DEGREES"),
                        help="Cut a doorway for an RDXArticulatedDoorObject at this pose in the object frame, on the lowest floor: "
                             "the frame origin on the hinge jamb, the opening along its +y, its +x pointing into the building as in EmptyRoom.json")
    parser.add_argument("--opening", type=float, nargs=3, action="append", metavar=("X", "Y", "YAW_DEGREES"),
                        help="Cut a doorway the size of --door's, with no door, centered at this point in the object frame on the "
                             "lowest floor, its passage along YAW_DEGREES (a multiple of 90). Repeat for more than one")
    parser.add_argument("--door-wall-depth", type=float, default=0.45,
                        help="How far the doorway is cut through the wall to each side of the door frame (m)")
    args = parser.parse_args()
    if (args.door or args.opening) and len(args.glb) > 1:
        parser.error("--door and --opening apply to one scene at a time")
    for glb in args.glb:
        convert(glb.expanduser().resolve(), args)


if __name__ == "__main__":
    main()
