"""
Create an articulated RDX environment object from a URDF.

The URDF's links are collapsed into rigid parts: every fixed joint is merged into its parent,
so each part that remains is either the root or the child of a movable joint (revolute,
continuous or prismatic). For each part this writes, into
``environmentObjects/<stem>/`` in the libgdx resources:

    <part>.glb                   visual, all of the part's visuals merged, in the part frame
    <part>_collision_<i>.stl     one file per collision piece, in the part frame
    <stem>.urdf                  the normalized URDF that points at those files
    <stem>_rdx.json              contact pairs MuJoCo should exclude (see below)

Like ``add_rdx_object_from_glb.py``, everything is Z-up and in meters, as RDX loads it.

Collision pieces are convex hulls by default, which is what MuJoCo collides with anyway, so the
collision drawn in RDX matches the simulated one. Keep the pieces split: a hollow part such as a
drawer is several convex pieces, and merging them into one hull fills it in.

Links without an <inertial> get one computed from their collision hulls at ``--density``.
``--part-mass NAME=KG`` rescales one part, for example a drawer whose collision is a solid box.

``<stem>_rdx.json`` lists contact pairs to exclude in MuJoCo:
  * every part and its parent across a movable joint. PhysX skips those contacts, and MuJoCo only
    skips parent-child contacts when the parent is not welded to the world;
  * with MuJoCo installed, any other pair already touching in the zero configuration, which is
    an overlap in the source asset rather than real contact.
With MuJoCo installed, the script then drives each joint to both limits and reports where it ends
up, to catch joints that are still blocked.

The Java class and its library registration are written only once
``RDXURDFArticulatedObject.java`` exists. Until then the resources are written and the Java
step is skipped.

Usage, from the ihmc-open-robotics-software repository:
    python3 scripts/add_rdx_articulated_object_from_urdf.py path/to/object.urdf
    python3 scripts/add_rdx_articulated_object_from_urdf.py cabinet.urdf --name "Kitchen Cabinet" \\
        --part-mass drawer_bottom=3 --density 600
"""

import argparse
import json
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import euler_from_matrix, euler_matrix

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.dont_write_bytecode = True  # importing the sibling script must not touch its tracked .pyc
from add_rdx_object_from_glb import (  # noqa: E402
    LIBRARY,
    OBJECTS_DIR,
    REPO,
    RESOURCES,
    java_string,
    pascal_case,
    register_factory,
    title_case,
)

BASE_CLASS = "RDXURDFArticulatedObject"
MOVABLE = {"revolute", "continuous", "prismatic"}
SUPPORTED = MOVABLE | {"fixed"}
MIN_MASS = 1.0e-3
TOUCH_TOLERANCE = 1.0e-4


@dataclass
class Geometry:
    """One <visual> or <collision> element, posed in its part's frame."""

    kind: str  # mesh, box, cylinder, sphere
    transform: np.ndarray
    mesh_path: Path | None = None
    scale: np.ndarray = field(default_factory=lambda: np.ones(3))
    size: np.ndarray | None = None  # box: full extents. cylinder: (radius, length). sphere: (radius,)
    rgba: np.ndarray | None = None


@dataclass
class Inertial:
    mass: float
    com: np.ndarray  # in the part frame
    inertia: np.ndarray  # 3x3 about the COM, in the part frame


@dataclass
class Part:
    name: str
    visuals: list[Geometry] = field(default_factory=list)
    collisions: list[Geometry] = field(default_factory=list)
    inertials: list[Inertial] = field(default_factory=list)
    merged: list[str] = field(default_factory=list)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    transform: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None
    damping: float | None
    friction: float | None


# ---------------------------------------------------------------------------------------------
# URDF parsing
# ---------------------------------------------------------------------------------------------


def floats(text: str | None, count: int, default: float = 0.0) -> np.ndarray:
    if text is None:
        return np.full(count, default)
    values = [float(value) for value in text.split()]
    if len(values) != count:
        raise SystemExit(f"Expected {count} numbers, got {text!r}")
    return np.array(values)


def origin_transform(element: ET.Element | None) -> np.ndarray:
    origin = element.find("origin") if element is not None else None
    if origin is None:
        return np.eye(4)
    xyz = floats(origin.get("xyz"), 3)
    rpy = floats(origin.get("rpy"), 3)
    transform = euler_matrix(*rpy, axes="sxyz")
    transform[:3, 3] = xyz
    return transform


def resolve_mesh(filename: str, urdf_dir: Path, packages: dict[str, Path]) -> Path:
    if filename.startswith("package://"):
        package, _, rest = filename[len("package://"):].partition("/")
        if package not in packages:
            raise SystemExit(f"{filename} needs --package-dir {package}=PATH")
        path = packages[package] / rest
    elif filename.startswith("file://"):
        path = Path(filename[len("file://"):])
    else:
        path = Path(filename)
        if not path.is_absolute():
            path = urdf_dir / path
    if not path.is_file():
        raise SystemExit(f"Mesh not found: {path}")
    return path


def material_colors(robot: ET.Element) -> dict[str, np.ndarray]:
    colors = {}
    for material in robot.findall("material"):
        color = material.find("color")
        if material.get("name") and color is not None:
            colors[material.get("name")] = floats(color.get("rgba"), 4, 1.0)
    return colors


def parse_geometry(element: ET.Element, urdf_dir: Path, packages: dict[str, Path], colors: dict[str, np.ndarray]) -> Geometry | None:
    geometry = element.find("geometry")
    if geometry is None or len(geometry) == 0:
        return None
    shape = geometry[0]
    transform = origin_transform(element)
    rgba = None
    material = element.find("material")
    if material is not None:
        color = material.find("color")
        if color is not None:
            rgba = floats(color.get("rgba"), 4, 1.0)
        elif material.get("name") in colors:
            rgba = colors[material.get("name")]

    if shape.tag == "mesh":
        return Geometry(
            "mesh",
            transform,
            mesh_path=resolve_mesh(shape.get("filename"), urdf_dir, packages),
            scale=floats(shape.get("scale"), 3, 1.0),
            rgba=rgba,
        )
    if shape.tag == "box":
        return Geometry("box", transform, size=floats(shape.get("size"), 3), rgba=rgba)
    if shape.tag == "cylinder":
        return Geometry("cylinder", transform, size=np.array([float(shape.get("radius")), float(shape.get("length"))]), rgba=rgba)
    if shape.tag == "sphere":
        return Geometry("sphere", transform, size=np.array([float(shape.get("radius"))]), rgba=rgba)
    print(f"  warning: skipping unsupported geometry <{shape.tag}>")
    return None


def parse_inertial(element: ET.Element) -> Inertial | None:
    inertial = element.find("inertial")
    if inertial is None or inertial.find("mass") is None:
        return None
    transform = origin_transform(inertial)
    values = inertial.find("inertia")
    local = np.zeros((3, 3))
    if values is not None:
        get = lambda key: float(values.get(key, 0.0))  # noqa: E731
        local = np.array(
            [
                [get("ixx"), get("ixy"), get("ixz")],
                [get("ixy"), get("iyy"), get("iyz")],
                [get("ixz"), get("iyz"), get("izz")],
            ]
        )
    rotation = transform[:3, :3]
    return Inertial(float(inertial.find("mass").get("value")), transform[:3, 3].copy(), rotation @ local @ rotation.T)


def parse_urdf(path: Path, packages: dict[str, Path]) -> tuple[dict[str, Part], list[Joint]]:
    robot = ET.parse(path).getroot()
    if robot.tag != "robot":
        raise SystemExit(f"{path} is not a URDF (root element is <{robot.tag}>)")
    colors = material_colors(robot)
    parts = {}
    for link in robot.findall("link"):
        part = Part(link.get("name"), merged=[link.get("name")])
        for visual in link.findall("visual"):
            geometry = parse_geometry(visual, path.parent, packages, colors)
            if geometry is not None:
                part.visuals.append(geometry)
        for collision in link.findall("collision"):
            geometry = parse_geometry(collision, path.parent, packages, colors)
            if geometry is not None:
                part.collisions.append(geometry)
        inertial = parse_inertial(link)
        if inertial is not None:
            part.inertials.append(inertial)
        parts[part.name] = part

    joints = []
    for element in robot.findall("joint"):
        kind = element.get("type")
        if kind not in SUPPORTED:
            raise SystemExit(f"Joint {element.get('name')} is {kind!r}. Supported types: {', '.join(sorted(SUPPORTED))}")
        limit = element.find("limit")
        dynamics = element.find("dynamics")
        optional = lambda node, key: float(node.get(key)) if node is not None and node.get(key) is not None else None  # noqa: E731
        axis = floats(element.find("axis").get("xyz"), 3) if element.find("axis") is not None else np.array([1.0, 0.0, 0.0])
        if np.linalg.norm(axis) == 0.0:
            raise SystemExit(f"Joint {element.get('name')} has a zero axis")
        joints.append(
            Joint(
                name=element.get("name"),
                type=kind,
                parent=element.find("parent").get("link"),
                child=element.find("child").get("link"),
                transform=origin_transform(element),
                axis=axis / np.linalg.norm(axis),
                lower=optional(limit, "lower"),
                upper=optional(limit, "upper"),
                effort=optional(limit, "effort"),
                velocity=optional(limit, "velocity"),
                damping=optional(dynamics, "damping"),
                friction=optional(dynamics, "friction"),
            )
        )
    for joint in joints:
        for link in (joint.parent, joint.child):
            if link not in parts:
                raise SystemExit(f"Joint {joint.name} names missing link {link!r}")
    children = [joint.child for joint in joints]
    if len(set(children)) != len(children):
        raise SystemExit("A link is the child of more than one joint, so this URDF is not a tree")
    roots = [name for name in parts if name not in children]
    if len(roots) != 1:
        raise SystemExit(f"Expected one root link, found {roots}")
    return parts, joints


# ---------------------------------------------------------------------------------------------
# Merging fixed joints
# ---------------------------------------------------------------------------------------------


def moved(geometry: Geometry, transform: np.ndarray) -> Geometry:
    return Geometry(geometry.kind, transform @ geometry.transform, geometry.mesh_path, geometry.scale, geometry.size, geometry.rgba)


def moved_inertial(inertial: Inertial, transform: np.ndarray) -> Inertial:
    rotation = transform[:3, :3]
    return Inertial(inertial.mass, rotation @ inertial.com + transform[:3, 3], rotation @ inertial.inertia @ rotation.T)


def merge_fixed_joints(parts: dict[str, Part], joints: list[Joint]) -> tuple[dict[str, Part], list[Joint]]:
    """Fold each fixed joint's child into its parent, moving the child's geometry and joints."""
    joints = list(joints)
    while True:
        fixed = next((joint for joint in joints if joint.type == "fixed"), None)
        if fixed is None:
            return parts, joints
        parent, child = parts[fixed.parent], parts.pop(fixed.child)
        parent.visuals += [moved(geometry, fixed.transform) for geometry in child.visuals]
        parent.collisions += [moved(geometry, fixed.transform) for geometry in child.collisions]
        parent.inertials += [moved_inertial(inertial, fixed.transform) for inertial in child.inertials]
        parent.merged += child.merged
        joints.remove(fixed)
        for joint in joints:
            if joint.parent == child.name:
                joint.parent = parent.name
                joint.transform = fixed.transform @ joint.transform


def ordered_parts(parts: dict[str, Part], joints: list[Joint]) -> list[str]:
    """Part names, root first, then each joint's child after its parent."""
    root = next(name for name in parts if name not in {joint.child for joint in joints})
    order, frontier = [], [root]
    while frontier:
        name = frontier.pop(0)
        order.append(name)
        frontier += [joint.child for joint in joints if joint.parent == name]
    return order


# ---------------------------------------------------------------------------------------------
# Meshes
# ---------------------------------------------------------------------------------------------


def primitive_mesh(geometry: Geometry) -> trimesh.Trimesh:
    if geometry.kind == "box":
        return trimesh.creation.box(extents=geometry.size)
    if geometry.kind == "cylinder":
        return trimesh.creation.cylinder(radius=geometry.size[0], height=geometry.size[1], sections=32)
    return trimesh.creation.icosphere(subdivisions=3, radius=geometry.size[0])


def scale_matrix(scale: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = np.diag(scale)
    return matrix


def visual_scene(part: Part) -> trimesh.Scene | None:
    scene = trimesh.Scene()
    for index, geometry in enumerate(part.visuals):
        if geometry.kind == "mesh":
            loaded = trimesh.load(str(geometry.mesh_path))
            pieces = loaded.dump() if isinstance(loaded, trimesh.Scene) else [loaded]
            base = geometry.transform @ scale_matrix(geometry.scale)
        else:
            pieces = [primitive_mesh(geometry)]
            base = geometry.transform
        for piece_index, piece in enumerate(pieces):
            if not isinstance(piece, trimesh.Trimesh) or len(piece.faces) == 0:
                continue
            piece = piece.copy()
            if geometry.rgba is not None and not hasattr(piece.visual, "material"):
                piece.visual.face_colors = (np.clip(geometry.rgba, 0.0, 1.0) * 255).astype(np.uint8)
            piece.apply_transform(base)
            scene.add_geometry(piece, geom_name=f"{part.name}_{index}_{piece_index}")
    return scene if scene.geometry else None


def collision_pieces(part: Part, convex: bool) -> list[trimesh.Trimesh]:
    pieces = []
    for geometry in part.collisions:
        if geometry.kind == "mesh":
            mesh = trimesh.load(str(geometry.mesh_path), force="mesh")
            mesh.apply_transform(geometry.transform @ scale_matrix(geometry.scale))
        else:
            mesh = primitive_mesh(geometry)
            mesh.apply_transform(geometry.transform)
        mesh.merge_vertices()
        mesh.update_faces(mesh.nondegenerate_faces())
        mesh.remove_unreferenced_vertices()
        if len(mesh.faces) == 0:
            continue
        pieces.append(mesh.convex_hull if convex else mesh)
    return pieces


def combined_inertial(inertials: list[Inertial]) -> Inertial | None:
    inertials = [inertial for inertial in inertials if inertial.mass > 0.0]
    if not inertials:
        return None
    mass = sum(inertial.mass for inertial in inertials)
    com = sum(inertial.mass * inertial.com for inertial in inertials) / mass
    inertia = np.zeros((3, 3))
    for inertial in inertials:
        offset = inertial.com - com
        inertia += inertial.inertia + inertial.mass * (offset @ offset * np.eye(3) - np.outer(offset, offset))
    return Inertial(mass, com, inertia)


def inertial_from_hulls(pieces: list[trimesh.Trimesh], density: float) -> Inertial | None:
    inertials = []
    for piece in pieces:
        hull = piece if piece.is_convex else piece.convex_hull
        if hull.volume <= 0.0:
            continue
        hull = hull.copy()
        hull.density = density
        inertials.append(Inertial(hull.mass, hull.center_mass.copy(), hull.moment_inertia.copy()))
    return combined_inertial(inertials)


def scaled_inertial(inertial: Inertial, mass: float) -> Inertial:
    return Inertial(mass, inertial.com, inertial.inertia * (mass / inertial.mass))


# ---------------------------------------------------------------------------------------------
# Output URDF
# ---------------------------------------------------------------------------------------------


def fmt(value: float) -> str:
    text = f"{value:.9g}"
    return "0" if text == "-0" else text


def vector(values) -> str:
    return " ".join(fmt(float(value)) for value in values)


def add_origin(parent: ET.Element, transform: np.ndarray) -> None:
    ET.SubElement(parent, "origin", xyz=vector(transform[:3, 3]), rpy=vector(euler_from_matrix(transform, axes="sxyz")))


def urdf_tree(stem: str, order: list[str], joints: list[Joint], files: dict, inertials: dict[str, Inertial], with_visuals: bool = True) -> ET.ElementTree:
    robot = ET.Element("robot", name=stem)
    for name in order:
        link = ET.SubElement(robot, "link", name=name)
        inertial = inertials.get(name)
        if inertial is not None:
            element = ET.SubElement(link, "inertial")
            ET.SubElement(element, "origin", xyz=vector(inertial.com), rpy="0 0 0")
            ET.SubElement(element, "mass", value=fmt(inertial.mass))
            i = inertial.inertia
            ET.SubElement(
                element,
                "inertia",
                ixx=fmt(i[0, 0]), ixy=fmt(i[0, 1]), ixz=fmt(i[0, 2]),
                iyy=fmt(i[1, 1]), iyz=fmt(i[1, 2]), izz=fmt(i[2, 2]),
            )
        if with_visuals and files[name]["visual"]:
            visual = ET.SubElement(link, "visual")
            ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=files[name]["visual"])
        for collision_file in files[name]["collisions"]:
            collision = ET.SubElement(link, "collision")
            ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(ET.SubElement(collision, "geometry"), "mesh", filename=collision_file)
    for joint in joints:
        element = ET.SubElement(robot, "joint", name=joint.name, type=joint.type)
        add_origin(element, joint.transform)
        ET.SubElement(element, "parent", link=joint.parent)
        ET.SubElement(element, "child", link=joint.child)
        ET.SubElement(element, "axis", xyz=vector(joint.axis))
        limit = {}
        if joint.type != "continuous" and joint.lower is not None and joint.upper is not None:
            limit.update(lower=fmt(joint.lower), upper=fmt(joint.upper))
        if joint.effort is not None:
            limit["effort"] = fmt(joint.effort)
        if joint.velocity is not None:
            limit["velocity"] = fmt(joint.velocity)
        if limit:
            ET.SubElement(element, "limit", **limit)
        dynamics = {}
        if joint.damping is not None:
            dynamics["damping"] = fmt(joint.damping)
        if joint.friction is not None:
            dynamics["friction"] = fmt(joint.friction)
        if dynamics:
            ET.SubElement(element, "dynamics", **dynamics)
    tree = ET.ElementTree(robot)
    ET.indent(tree)
    return tree


# ---------------------------------------------------------------------------------------------
# MuJoCo checks
# ---------------------------------------------------------------------------------------------


def mujoco_model(mujoco, directory: Path, tree: ET.ElementTree, excludes: list[tuple[str, str]]):
    """Compile the collision-only URDF in MuJoCo, then add the exclusions through an MJCF round trip."""
    root = ET.fromstring(ET.tostring(tree.getroot()))
    compiler = ET.Element("mujoco")
    ET.SubElement(compiler, "compiler", meshdir=str(directory), strippath="false", fusestatic="false", discardvisual="false")
    root.insert(0, compiler)
    with tempfile.TemporaryDirectory() as scratch:
        urdf = Path(scratch) / "check.urdf"
        urdf.write_text(ET.tostring(root, encoding="unicode"))
        model = mujoco.MjModel.from_xml_path(str(urdf))
        if not excludes:
            return model
        mjcf = Path(scratch) / "check.xml"
        mujoco.mj_saveLastXML(str(mjcf), model)
        xml = ET.parse(mjcf).getroot()
        contact = xml.find("contact")
        if contact is None:
            contact = ET.SubElement(xml, "contact")
        for body1, body2 in excludes:
            ET.SubElement(contact, "exclude", body1=body1, body2=body2)
        compiler_element = xml.find("compiler")
        if compiler_element is not None:
            compiler_element.set("meshdir", str(directory))
        mjcf.write_text(ET.tostring(xml, encoding="unicode"))
        return mujoco.MjModel.from_xml_path(str(mjcf))


def joint_stops(joint: Joint) -> list[float]:
    """Positions worth testing: both limits and zero, or a few angles for a continuous joint."""
    if joint.lower is None or joint.upper is None:
        return [0.0, np.pi / 2.0, np.pi, -np.pi / 2.0]
    return sorted({joint.lower, joint.upper, min(max(0.0, joint.lower), joint.upper)})


def sample_configurations(joints: list[Joint], limit: int = 729, seed: int = 0) -> list[dict[str, float]]:
    """Every combination of joint stops when that is at most ``limit``, otherwise a random subset."""
    stops = [joint_stops(joint) for joint in joints]
    count = int(np.prod([len(values) for values in stops])) if stops else 1
    if count <= limit:
        grid = np.array(np.meshgrid(*stops, indexing="ij")).reshape(len(joints), -1).T if stops else np.zeros((1, 0))
    else:
        rng = np.random.default_rng(seed)
        grid = np.array([[rng.choice(values) for values in stops] for _ in range(limit)])
    return [{joint.name: float(value) for joint, value in zip(joints, row)} for row in grid]


def penetrations(mujoco, model, joints: list[Joint], configurations: list[dict[str, float]]) -> dict[tuple[str, str], tuple[float, dict]]:
    """Deepest penetration between two different parts over the configurations, posed kinematically."""
    data = mujoco.MjData(model)
    deepest: dict[tuple[str, str], tuple[float, dict]] = {}
    for configuration in configurations:
        data.qpos[:] = 0.0
        for name, value in configuration.items():
            data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
        mujoco.mj_forward(model, data)
        for index in range(data.ncon):
            contact = data.contact[index]
            if contact.dist > -TOUCH_TOLERANCE:
                continue
            body1 = model.body(model.geom_bodyid[contact.geom1]).name
            body2 = model.body(model.geom_bodyid[contact.geom2]).name
            if body1 == body2 or "world" in (body1, body2):
                continue
            key = tuple(sorted((body1, body2)))
            depth = -float(contact.dist)
            if depth > deepest.get(key, (0.0, None))[0]:
                deepest[key] = (depth, dict(configuration))
    return deepest


def drive(mujoco, model, joints: list[Joint], targets: dict[str, float], steps: int = 3000) -> dict[str, float]:
    """Servo every joint to its target with gravity off, so contact is the only thing that can stop one."""
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    ids = {joint.name: model.joint(joint.name).id for joint in joints}
    masses = {name: max(float(model.body_subtreemass[model.jnt_bodyid[i]]), MIN_MASS) for name, i in ids.items()}
    for _ in range(steps):
        for name, joint_id in ids.items():
            qpos, dof = model.jnt_qposadr[joint_id], model.jnt_dofadr[joint_id]
            gain = 400.0 * masses[name]
            data.qfrc_applied[dof] = gain * (targets[name] - data.qpos[qpos]) - 2.0 * np.sqrt(gain * masses[name]) * data.qvel[dof]
        mujoco.mj_step(model, data)
    return {name: float(data.qpos[model.jnt_qposadr[joint_id]]) for name, joint_id in ids.items()}


def limit_checks(mujoco, model, joints: list[Joint]) -> list[tuple[str, str, float, float, bool]]:
    """
    Opens each joint to both stops with the others held closed, then held open. "Closed" is each
    joint's stop nearest zero and "open" its other limit. Returns (joint, others, target, reached, ok).
    """
    limited = [joint for joint in joints if joint.lower is not None and joint.upper is not None]
    closed = {joint.name: min(max(0.0, joint.lower), joint.upper) for joint in limited}
    opened = {joint.name: joint.upper if abs(joint.upper - closed[joint.name]) >= abs(joint.lower - closed[joint.name]) else joint.lower for joint in limited}
    results = []
    for others_name, others in (("closed", closed), ("open", opened)):
        for joint in limited:
            for target in (joint.lower, joint.upper):
                targets = dict(others)
                targets[joint.name] = target
                reached = drive(mujoco, model, limited, targets)[joint.name]
                ok = abs(reached - target) <= 0.02 * (joint.upper - joint.lower)
                results.append((joint.name, others_name, target, reached, ok))
    return results


# ---------------------------------------------------------------------------------------------
# Java
# ---------------------------------------------------------------------------------------------


def class_source(class_name: str, display_name: str, urdf_resource: str) -> str:
    return f"""package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.simulation.environment.object.{BASE_CLASS};

public class {class_name} extends {BASE_CLASS}
{{
   public static final String NAME = "{java_string(display_name)}";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, {class_name}.class);

   public {class_name}()
   {{
      super(NAME, FACTORY, "{urdf_resource}");
   }}
}}
"""


def base_class_path() -> Path | None:
    matches = list((REPO / "ihmc-high-level-behaviors").rglob(f"{BASE_CLASS}.java"))
    return matches[0] if matches else None


# ---------------------------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------------------------


def parse_key_values(items: list[str], what: str, convert) -> dict:
    result = {}
    for item in items:
        key, sep, value = item.partition("=")
        if not sep or not key:
            raise SystemExit(f"{what} must look like NAME=VALUE, got {item!r}")
        result[key] = convert(value)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Create an articulated RDX object and its MuJoCo collision from a URDF")
    parser.add_argument("urdf", help="URDF file. Mesh paths resolve relative to it.")
    parser.add_argument("--name", default=None, help="Display name. Defaults to the URDF file name.")
    parser.add_argument("--stem", default=None, help="Resource folder and URDF name. Defaults to the URDF file name.")
    parser.add_argument("--density", type=float, default=1000.0, help="kg/m^3 for links without <inertial> (default 1000)")
    parser.add_argument("--part-mass", action="append", default=[], metavar="PART=KG", help="Set one part's mass. Repeatable.")
    parser.add_argument("--package-dir", action="append", default=[], metavar="PKG=PATH", help="Where package://PKG/ resolves. Repeatable.")
    parser.add_argument("--no-convex", action="store_true", help="Keep collision surfaces instead of their convex hulls")
    parser.add_argument("--no-check", action="store_true", help="Skip the MuJoCo overlap and joint-limit checks")
    parser.add_argument("--overlap-tolerance", type=float, default=3.0, metavar="MM",
                        help="Overlaps between unjointed parts up to this depth are excluded as asset artifacts (default 3 mm)")
    parser.add_argument("--force", action="store_true", help="Replace an existing resource folder and class")
    parser.add_argument("--dry-run", action="store_true", help="Report what would be written without writing anything")
    args = parser.parse_args()

    src = Path(args.urdf).expanduser().resolve()
    if not src.is_file():
        raise SystemExit(f"URDF not found: {src}")
    if args.density <= 0.0:
        raise SystemExit("--density must be positive")
    stem = args.stem or src.stem
    if not re.fullmatch(r"[A-Za-z0-9_-]+", stem):
        raise SystemExit(f"Stem {stem!r} must contain only letters, digits, '_' and '-'. Pass --stem.")
    display_name = args.name or title_case(stem)
    class_name = "RDX" + pascal_case(args.name if args.name else stem) + "Object"
    if not re.fullmatch(r"RDX[A-Za-z0-9_]+Object", class_name):
        raise SystemExit(f"Cannot build a Java class name from {display_name!r}")
    packages = {key: Path(value).expanduser().resolve() for key, value in parse_key_values(args.package_dir, "--package-dir", str).items()}
    part_masses = parse_key_values(args.part_mass, "--part-mass", float)

    parts, joints = parse_urdf(src, packages)
    link_count = len(parts)
    parts, joints = merge_fixed_joints(parts, joints)
    order = ordered_parts(parts, joints)
    for name in part_masses:
        if name not in parts:
            raise SystemExit(f"--part-mass names {name!r}, which is not a part. Parts: {', '.join(order)}")
    print(f"{src.name}: {link_count} links -> {len(order)} parts, {len(joints)} movable joints")

    resource_dir = RESOURCES / "environmentObjects" / stem
    if resource_dir.exists() and not args.force and not args.dry_run:
        raise SystemExit(f"{resource_dir} already exists. Pass --force to replace it.")

    staging = Path(tempfile.mkdtemp(prefix=f"{stem}_"))
    try:
        files, inertials, rows = {}, {}, []
        for name in order:
            part = parts[name]
            scene = visual_scene(part)
            visual_file = None
            if scene is not None:
                visual_file = f"{name}.glb"
                # With normals: without them the renderer cannot shade curved surfaces smoothly
                scene.export(str(staging / visual_file), file_type="glb", include_normals=True)
            pieces = collision_pieces(part, convex=not args.no_convex)
            collision_files = []
            for index, piece in enumerate(pieces):
                collision_files.append(f"{name}_collision_{index}.stl")
                piece.export(str(staging / collision_files[-1]))
            files[name] = {"visual": visual_file, "collisions": collision_files}

            inertial, source = combined_inertial(part.inertials), "urdf"
            if inertial is None:
                inertial, source = inertial_from_hulls(pieces, args.density), f"hulls @ {fmt(args.density)} kg/m^3"
            if inertial is None:
                inertial, source = Inertial(MIN_MASS, np.zeros(3), np.eye(3) * 1.0e-6), "placeholder (no collision)"
            if name in part_masses:
                inertial, source = scaled_inertial(inertial, part_masses[name]), "--part-mass"
            inertials[name] = inertial
            joint = next((j for j in joints if j.child == name), None)
            rows.append((name, joint.type if joint else "root", inertial.mass, source, len(collision_files), "+".join(part.merged[1:])))

        print(f"  {'part':<16} {'joint':<10} {'mass kg':>8}  {'pieces':>6}  mass from / merged links")
        for name, kind, mass, source, count, merged in rows:
            extra = f"; merged {merged}" if merged else ""
            print(f"  {name:<16} {kind:<10} {mass:>8.2f}  {count:>6}  {source}{extra}")
        for joint in joints:
            limits = "unlimited" if joint.lower is None or joint.upper is None else f"[{fmt(joint.lower)}, {fmt(joint.upper)}]"
            print(f"  joint {joint.name}: {joint.type} {joint.parent} -> {joint.child}, axis {vector(joint.axis)}, {limits}")

        tree = urdf_tree(stem, order, joints, files, inertials)
        tree.write(staging / f"{stem}.urdf", encoding="unicode", xml_declaration=True)

        excludes = [(joint.parent, joint.child, "movable joint") for joint in joints]
        if not args.no_check:
            try:
                import mujoco
            except ImportError:
                mujoco = None
                print("  MuJoCo is not installed, so overlaps were not checked. Only jointed pairs are excluded.")
            if mujoco is not None:
                collision_tree = urdf_tree(stem, order, joints, files, inertials, with_visuals=False)
                jointed = [(a, b) for a, b, _ in excludes]
                model = mujoco_model(mujoco, staging, collision_tree, jointed)
                configurations = sample_configurations(joints)
                tolerance = args.overlap_tolerance / 1000.0
                for (body1, body2), (depth, where) in sorted(penetrations(mujoco, model, joints, configurations).items()):
                    pose = ", ".join(f"{name}={fmt(value)}" for name, value in where.items() if value != 0.0) or "all joints at zero"
                    if depth <= tolerance:
                        excludes.append((body1, body2, f"shallow overlap, {depth * 1000:.1f} mm at {pose}"))
                        print(f"  overlap {body1} <-> {body2}: at most {depth * 1000:.1f} mm ({pose}), excluded as an asset artifact")
                    else:
                        print(f"  overlap {body1} <-> {body2}: up to {depth * 1000:.1f} mm ({pose}), kept. These parts block each other.")
                model = mujoco_model(mujoco, staging, collision_tree, [(a, b) for a, b, _ in excludes])
                print(f"  MuJoCo {mujoco.__version__}: {model.nbody - 1} bodies, {model.njnt} joints, {model.ngeom} geoms, "
                      f"{len(configurations)} configurations checked")
                blocked = []
                for name, others, target, reached, ok in limit_checks(mujoco, model, joints):
                    if not ok:
                        blocked.append(f"{name} -> {fmt(target)} (reached {reached:.3f}) with the others {others}")
                checks = 2 * 2 * len([joint for joint in joints if joint.lower is not None and joint.upper is not None])
                print(f"  joint travel: {checks - len(blocked)}/{checks} moves reached their limit")
                for line in blocked:
                    print(f"    blocked: {line}")

        sidecar = {
            "urdf": f"{stem}.urdf",
            "excludeContacts": [{"body1": a, "body2": b, "reason": reason} for a, b, reason in excludes],
        }
        (staging / f"{stem}_rdx.json").write_text(json.dumps(sidecar, indent=2) + "\n")

        urdf_resource = f"environmentObjects/{stem}/{stem}.urdf"
        class_path = OBJECTS_DIR / f"{class_name}.java"
        base = base_class_path()
        written = sorted(path.name for path in staging.iterdir())
        print(f"  resources: environmentObjects/{stem}/ ({len(written)} files)")
        if args.dry_run:
            print("  dry-run: nothing was written")
            return

        if resource_dir.exists():
            shutil.rmtree(resource_dir)
        resource_dir.mkdir(parents=True)
        for name in written:
            shutil.copy2(staging / name, resource_dir / name)

        if base is None:
            print(f"  java: skipped, {BASE_CLASS}.java does not exist yet")
            return
        if class_path.exists() and not args.force:
            raise SystemExit(f"{class_path.name} already exists. Pass --force to replace it.")
        class_path.write_text(class_source(class_name, display_name, urdf_resource))
        print(f"  class:     {class_path.relative_to(REPO)}")
        register_factory(class_name, dry_run=False)
    finally:
        shutil.rmtree(staging, ignore_errors=True)


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        sys.exit(0)
