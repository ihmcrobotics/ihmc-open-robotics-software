"""
Create an RDX environment object from a visual GLB.

Copies the GLB into the environment-object resources, writes a MuJoCo collision
STL beside it, generates the RDX object class, and registers that factory in
RDXEnvironmentObjectLibrary.

The collision file is always ``<stem>_collision.stl`` beside the visual. Without
``--convex`` that STL is the visual surface. With ``--convex`` it is a convex
hull, optionally pre-simplified with MeshLab first, the same approach as
``ihmc-alex-sdk/utils/make_convex_hulls.py``.

Usage, from the ihmc-open-robotics-software repository:
    python3 scripts/add_rdx_object_from_glb.py path/to/visual.glb
    python3 scripts/add_rdx_object_from_glb.py path/to/visual.glb --convex
    python3 scripts/add_rdx_object_from_glb.py path/to/visual.glb --convex --mlx path/to/reduce.mlx --name "Tool Box" --mass 4.5
"""

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh

SCRIPT_DIR = Path(__file__).resolve().parent
REPO = SCRIPT_DIR.parent
RESOURCES = REPO / "ihmc-high-level-behaviors" / "src" / "libgdx" / "resources"
OBJECTS_DIR = (
    REPO
    / "ihmc-high-level-behaviors"
    / "src"
    / "libgdx"
    / "java"
    / "us"
    / "ihmc"
    / "rdx"
    / "simulation"
    / "environment"
    / "object"
    / "objects"
)
LIBRARY = OBJECTS_DIR.parent / "RDXEnvironmentObjectLibrary.java"
MIN_EXTENT = 0.001


def split_words(stem: str) -> list[str]:
    parts = []
    for token in stem.replace("-", "_").split("_"):
        parts.extend(re.findall(r"[A-Z]+(?![a-z])|[A-Z]?[a-z]+|\d+", token))
    return [part for part in parts if part] or [stem]


def pascal_case(stem: str) -> str:
    return "".join(part[:1].upper() + part[1:] for part in split_words(stem))


def title_case(stem: str) -> str:
    return " ".join(part[:1].upper() + part[1:] for part in split_words(stem))


def java_double(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    if "." not in text:
        text += ".0"
    return text


def java_float(value: float) -> str:
    return java_double(value) + "f"


def java_string(value: str) -> str:
    return value.replace("\\", "\\\\").replace("\"", "\\\"")


def simplify_with_meshlab(input_path: Path, script: Path) -> Path:
    tmp = tempfile.NamedTemporaryFile(suffix=".stl", delete=False)
    tmp.close()
    subprocess.run(
        ["meshlabserver", "-i", str(input_path), "-o", tmp.name, "-s", str(script)],
        check=True,
        capture_output=True,
    )
    return Path(tmp.name)


def convex_hull_from_mesh(input_path: Path, mlx: Path | None) -> trimesh.Trimesh:
    source = input_path
    if mlx is not None:
        source = simplify_with_meshlab(input_path, mlx)

    mesh = trimesh.load(str(source), force="mesh")
    mesh.merge_vertices()
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()

    if mlx is not None:
        source.unlink()

    return mesh.convex_hull


def load_visual(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(str(path), force="mesh")
    if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) == 0:
        raise SystemExit(f"No triangles in {path}")
    loaded.merge_vertices()
    loaded.update_faces(loaded.nondegenerate_faces())
    loaded.update_faces(loaded.unique_faces())
    loaded.remove_unreferenced_vertices()
    if len(loaded.faces) == 0:
        raise SystemExit(f"No triangles left after cleanup in {path}")
    return loaded


def collision_mesh(visual_path: Path, convex: bool, mlx: Path | None) -> trimesh.Trimesh:
    if convex:
        return convex_hull_from_mesh(visual_path, mlx)
    if mlx is not None:
        raise SystemExit("--mlx is only used with --convex")
    return load_visual(visual_path)


def safe_stem(path: Path) -> str:
    stem = path.stem
    if not re.fullmatch(r"[A-Za-z0-9_-]+", stem):
        raise SystemExit(
            f"GLB name {stem!r} must contain only letters, digits, '_' and '-'. Rename the file first."
        )
    return stem


def class_source(class_name: str, display_name: str, resource_path: str, mesh: trimesh.Trimesh, mass: float) -> str:
    extents = np.maximum(mesh.bounds[1] - mesh.bounds[0], MIN_EXTENT)
    center = (mesh.bounds[0] + mesh.bounds[1]) * 0.5
    radius = float(np.linalg.norm(extents) * 0.5)
    size_x, size_y, size_z = (java_double(float(value)) for value in extents)
    center_x, center_y, center_z = (java_double(float(value)) for value in center)
    return f"""package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class {class_name} extends RDXEnvironmentObject
{{
   public static final String NAME = "{java_string(display_name)}";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, {class_name}.class);

   public {class_name}()
   {{
      super(NAME, FACTORY);
      loadRealisticModel("{resource_path}");

      setMass({java_float(mass)});
      Box3D collisionBox = new Box3D({size_x}, {size_y}, {size_z});
      collisionBox.getPosition().set({center_x}, {center_y}, {center_z});
      getBoundingSphere().setRadius({java_double(radius)});
      getBoundingSphere().getPosition().set({center_x}, {center_y}, {center_z});
      setCollisionGeometryObject(collisionBox);
   }}
}}
"""


def register_factory(class_name: str, dry_run: bool) -> None:
    text = LIBRARY.read_text()
    line = f"objectFactories.add({class_name}.FACTORY);"
    if line in text:
        print(f"  library already registers {class_name}")
        return
    lines = text.splitlines(keepends=True)
    last = max((index for index, existing in enumerate(lines) if "objectFactories.add(" in existing), default=None)
    if last is None:
        raise SystemExit(f"Could not find an objectFactories.add line in {LIBRARY}")
    indent = lines[last][: len(lines[last]) - len(lines[last].lstrip())]
    if dry_run:
        print(f"  would register {class_name}.FACTORY")
        return
    lines.insert(last + 1, f"{indent}{line}\n")
    LIBRARY.write_text("".join(lines))
    print(f"  registered {class_name}.FACTORY")


def write_outputs(src: Path, mesh: trimesh.Trimesh, class_name: str, display_name: str, mass: float, force: bool, dry_run: bool) -> None:
    stem = safe_stem(src)
    resource_dir = RESOURCES / "environmentObjects" / stem
    dest_glb = resource_dir / src.name
    dest_stl = resource_dir / f"{stem}_collision.stl"
    class_path = OBJECTS_DIR / f"{class_name}.java"
    resource_path = f"environmentObjects/{stem}/{src.name}"

    for required in (RESOURCES, OBJECTS_DIR, LIBRARY):
        if not required.exists():
            raise SystemExit(f"Missing {required}")
    if class_path.exists() and not force:
        raise SystemExit(f"{class_path.name} already exists. Pass --force to replace it.")
    if dest_glb.exists() and src.resolve() != dest_glb.resolve() and not force:
        raise SystemExit(f"{dest_glb} already exists. Pass --force to replace it.")

    print(f"  visual:    {resource_path}")
    print(f"  collision: environmentObjects/{stem}/{dest_stl.name}  ({len(mesh.faces)} faces)")
    print(f"  class:     {class_path.relative_to(REPO)}")
    if dry_run:
        print("  dry-run: class, collision mesh, and library were not written")
        if not class_path.exists():
            print(f"  missing class file: {class_path.name}")
        register_factory(class_name, dry_run=True)
        return

    resource_dir.mkdir(parents=True, exist_ok=True)
    if src.resolve() != dest_glb.resolve():
        shutil.copy2(src, dest_glb)
    mesh.export(dest_stl)
    class_path.write_text(class_source(class_name, display_name, resource_path, mesh, mass))
    register_factory(class_name, dry_run=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create an RDX object and MuJoCo collision mesh from a visual GLB")
    parser.add_argument("glb", help="Visual GLB (or glTF) mesh")
    parser.add_argument("--convex", action="store_true", help="Write a convex hull instead of the visual surface")
    parser.add_argument("--mlx", default=None, help="MeshLab script to pre-simplify the source before --convex")
    parser.add_argument("--name", default=None, help="Display name. Defaults to the file name.")
    parser.add_argument("--mass", type=float, default=1.0, help="Object mass in kilograms (default 1)")
    parser.add_argument("--force", action="store_true", help="Replace an existing class, visual, and collision mesh")
    parser.add_argument("--dry-run", action="store_true", help="Print the files that would be written")
    args = parser.parse_args()

    src = Path(args.glb).expanduser().resolve()
    if not src.is_file():
        raise SystemExit(f"GLB not found: {src}")
    if src.suffix.lower() not in {".glb", ".gltf"}:
        raise SystemExit(f"Expected a .glb or .gltf file, got {src.name}")
    if args.mass <= 0:
        raise SystemExit("--mass must be positive")
    if args.name is not None and not args.name.strip():
        raise SystemExit("--name must not be empty")

    stem = safe_stem(src)
    display_name = args.name if args.name else title_case(stem)
    class_name = "RDX" + pascal_case(args.name if args.name else stem) + "Object"
    if not re.fullmatch(r"RDX[A-Za-z0-9_]+Object", class_name):
        raise SystemExit(f"Cannot build a Java class name from {display_name!r}")

    mlx = Path(args.mlx).expanduser().resolve() if args.mlx else None
    if mlx is not None and not mlx.is_file():
        raise SystemExit(f"MeshLab script not found: {mlx}")

    mesh = collision_mesh(src, args.convex, mlx)
    kind = "convex hull" if args.convex else "visual surface"
    status = "convex" if mesh.is_convex else "non-convex"
    print(f"{src.name}: {len(mesh.faces)} faces, {kind}, {status}")
    write_outputs(src, mesh, class_name, display_name, args.mass, args.force, args.dry_run)


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        sys.exit(0)
