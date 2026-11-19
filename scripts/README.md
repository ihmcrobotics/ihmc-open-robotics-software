# Add an RDX object from a GLB

`add_rdx_object_from_glb.py` turns a visual GLB into an environment object the simulation can place and collide with.

From the `ihmc-open-robotics-software` repository:

```bash
python3 scripts/add_rdx_object_from_glb.py path/to/visual.glb
```

That does four things:

1. Copies the GLB to `ihmc-high-level-behaviors/src/libgdx/resources/environmentObjects/<stem>/<stem>.glb`.
2. Writes a MuJoCo collision mesh beside it as `<stem>_collision.stl`. The object loader looks up that name. `--convex` changes the mesh, not the file name.
3. Creates `RDX<Name>Object` under `ihmc-high-level-behaviors/src/libgdx/java/us/ihmc/rdx/simulation/environment/object/objects/`.
4. Registers that object's factory in `RDXEnvironmentObjectLibrary`.

The class uses a box fitted to the mesh as the primitive collision. When the STL is present, the loader uses that mesh instead.

The GLB file name may contain only letters, digits, `_`, and `-`. `tool_box.glb` is valid. `tool box.glb` is not.

## Convex collision

Without `--convex`, `<stem>_collision.stl` is the visual surface. With `--convex`, the same file is a convex hull of that mesh.

```bash
python3 scripts/add_rdx_object_from_glb.py path/to/visual.glb --convex
```

`--mlx` runs a MeshLab script on the visual before the hull, which keeps the face count down. This needs `meshlabserver` on `PATH`, and it is only valid together with `--convex`.

```bash
python3 scripts/add_rdx_object_from_glb.py path/to/visual.glb --convex --mlx path/to/reduce.mlx
```

## Concave objects: convex pieces

MuJoCo collides every mesh geom as the mesh's convex hull, with or without `--convex`. That works for a box or a bottle. For a concave object, the hull fills the gaps and contact happens in empty space. The RDX collision graphic draws the STL itself, so it can look right while the physics is not.

The door lever is an example. Its hull fills the space between the bar and the round base, so the fingers stopped up to 33 mm before they touched the bar.

`make_convex_collision_parts.py` cuts the collision mesh with axis-aligned planes and writes the convex hull of each piece as `<stem>_collision_convex_<i>.stl` beside `<stem>_collision.stl`. `RDXSimpleObject.findConvexCollisionPartFiles` finds them, from index 0 up to the first missing one. The articulated door uses them for the lever when they are present, and falls back to the single mesh when they are not. Other objects still use only `<stem>_collision.stl` until their MuJoCo export reads the pieces too.

### 1. See how far the single hull is from the surface

```bash
python3 scripts/make_convex_collision_parts.py \
  ihmc-high-level-behaviors/src/libgdx/resources/environmentObjects/door_handle/door_handle_collision.stl
```

This prints the mesh bounds in its own frame, and how far the hull is from the surface. For the lever, the median is 8 mm and the maximum is 32 mm.

### 2. Choose the pieces

Each `--piece` is a box of limits on the mesh axes: `x<a`, `x>a`, or `a<x<b`, joined with commas. The boxes should cover the whole mesh and not overlap. Write `--piece="..."` with `=`, because a limit that starts with `-` otherwise reads as an option.

To find good planes, cut first where the shape changes direction, then split long, curved parts into short sections. The lever is a round base (`y<-0.0175`), a neck (`-0.0175<y<0.014`), and a bar along +x, which is cut into five sections:

```bash
python3 scripts/make_convex_collision_parts.py \
  ihmc-high-level-behaviors/src/libgdx/resources/environmentObjects/door_handle/door_handle_collision.stl \
  --report \
  --piece="y<-0.0175" --piece="-0.0175<y<0.014" \
  --piece="y>0.014,x<0.025" --piece="y>0.014,0.025<x<0.045" --piece="y>0.014,0.045<x<0.065" \
  --piece="y>0.014,0.065<x<0.085" --piece="y>0.014,x>0.085"
```

`--report` prints, for each piece, how far its hull reaches outside the mesh. It also prints the total extra volume and any part of the mesh no piece covers. For the lever, the bar pieces are within 0.7 mm, and the extra volume drops from 273% to 25% of the lever. If a piece is far from the surface, cut it again where the gap is. The report samples random points, so it takes about a minute for a mesh with 20,000 triangles.

### 3. Write the pieces

The same command without `--report` writes the pieces. It first deletes any old `<stem>_collision_convex_*.stl`, so a run with fewer pieces leaves no stale files. Use `--dry-run` to list what it would write and remove.

The input must be a closed mesh for the report to be accurate. The cuts work on any mesh.

## Other options

```bash
python3 scripts/add_rdx_object_from_glb.py path/to/tool_box.glb --convex --name "Tool Box" --mass 4.5
```

| Option | Effect |
| --- | --- |
| `--name` | Display name in the environment builder. Also sets the class name, so `"Tool Box"` becomes `RDXToolBoxObject`. The resource folder still follows the file name. |
| `--mass` | Mass in kilograms. Default is `1`. |
| `--force` | Replace an existing visual, collision mesh, and class. |
| `--dry-run` | Print the paths and the library registration without writing files. |

## Check against the bucket

`bucket.glb` is already in the repository. This command only prints the paths, including `bucket_collision.stl`, because `RDXBucketObject.java` is already there:

```bash
python3 scripts/add_rdx_object_from_glb.py \
  ihmc-high-level-behaviors/src/libgdx/resources/environmentObjects/bucket/bucket.glb \
  --convex --name Bucket --mass 2 --dry-run
```

# Add an articulated RDX object from a URDF

`add_rdx_articulated_object_from_urdf.py` turns a URDF with movable joints (a cabinet with drawers and doors, a car door) into an environment object. RDX draws its parts at their joint angles, and MuJoCo simulates its joints, so the robot can open it.

A GLB cannot do this. It keeps meshes and their hierarchy, but no joints, limits, masses or collision shapes.

From the `ihmc-open-robotics-software` repository:

```bash
python3 scripts/add_rdx_articulated_object_from_urdf.py path/to/object.urdf
```

The script first merges every fixed joint into its parent link. Each link left is a rigid part: the root, or the child of a revolute, continuous or prismatic joint. A handle bolted to a drawer becomes part of that drawer. Then it writes these to `ihmc-high-level-behaviors/src/libgdx/resources/environmentObjects/<stem>/`:

| File | Contents |
| --- | --- |
| `<part>.glb` | All of the part's visuals merged into one GLB, Z-up, in the part's frame |
| `<part>_collision_<i>.stl` | One convex hull for each collision element of the part, in the part's frame |
| `<stem>.urdf` | The merged URDF, pointing at those files, with an `<inertial>` for every part |
| `<stem>_rdx.json` | Body pairs MuJoCo must not collide (see below) |

Finally it creates `RDX<Name>Object` and registers it in `RDXEnvironmentObjectLibrary`, as the GLB script does. The class only names the object and its URDF. `RDXURDFArticulatedObject` does the rest.

Each collision element stays a separate hull, because MuJoCo collides every mesh as its convex hull. A hollow part, such as a drawer, is several pieces. One hull would fill it in. Pass `--no-convex` to keep the surfaces instead. MuJoCo still collides them as hulls.

## Example: the Sektion cabinet from Isaac Sim

The Sektion cabinet in Isaac Sim's assets (`/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd`) has two drawers on prismatic joints and two doors on revolute joints. It is in the repository as `environmentObjects/sektion_cabinet/` and `RDXSektionCabinetObject`.

### 1. Export the URDF from Isaac Sim

Isaac Sim's URDF exporter (`isaacsim.asset.exporter.urdf`) writes the URDF and one OBJ per mesh. In the GUI it is in the **File** menu, with the other exporters. Headless, save this as `export_cabinet_urdf.py` and run it with Isaac Sim's Python:

```python
import os, sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.exporter.urdf")
app.update()
from pxr import Usd
from isaacsim.storage.native import get_assets_root_path
from nvidia.srl.from_usd.to_urdf import UsdToUrdf

out = sys.argv[1]
stage = Usd.Stage.Open(get_assets_root_path() + "/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd")
# The exporter does not see meshes inside instances, so turn instancing off and flatten first
for prim in stage.Traverse():
    if prim.IsInstance():
        prim.SetInstanceable(False)
flat = Usd.Stage.Open(stage.Flatten())
os.makedirs(out, exist_ok=True)
print(UsdToUrdf(flat).save_to_file(os.path.join(out, "sektion_cabinet.urdf")))
os._exit(0)
```

```bash
cd IsaacSim/_build/linux-x86_64/release
./python.sh export_cabinet_urdf.py ~/mujoco_assets/sektion_cabinet
```

Run it outside any conda environment that sets `PYTHONPATH`, such as Isaac Lab's. Isaac Sim otherwise imports that environment's packages and fails to start some extensions.

The result is `sektion_cabinet.urdf` and a `meshes/` folder of OBJ files. It has no `<inertial>` tags, so the next step computes masses.

### 2. Create the RDX object

```bash
python3 scripts/add_rdx_articulated_object_from_urdf.py ~/mujoco_assets/sektion_cabinet/sektion_cabinet.urdf
```

```
sektion_cabinet.urdf: 10 links -> 5 parts, 4 movable joints
  part             joint       mass kg  pieces  mass from / merged links
  cabinet          root          69.90       7  hulls @ 1000 kg/m^3; merged sektion
  door_left_link   revolute       3.89       2  hulls @ 1000 kg/m^3; merged door_left_nob_link
  door_right_link  revolute       3.89       2  hulls @ 1000 kg/m^3; merged door_right_nob_link
  drawer_bottom    prismatic     43.43       2  hulls @ 1000 kg/m^3; merged drawer_handle_bottom
  drawer_top       prismatic      5.33       8  hulls @ 1000 kg/m^3; merged drawer_handle_top
  joint door_left_joint: revolute cabinet -> door_left_link, axis 1 0 0, [-1.57, 0]
  joint door_right_joint: revolute cabinet -> door_right_link, axis 1 0 0, [0, 1.57]
  joint drawer_bottom_joint: prismatic cabinet -> drawer_bottom, axis 1 0 0, [0, 0.4]
  joint drawer_top_joint: prismatic cabinet -> drawer_top, axis 1 0 0, [0, 0.4]
  overlap door_right_link <-> drawer_bottom: at most 0.5 mm (door_left_joint=-1.57, door_right_joint=1.57, drawer_bottom_joint=0.4), excluded as an asset artifact
  MuJoCo 3.3.7: 5 bodies, 4 joints, 21 geoms, 16 configurations checked
  joint travel: 16/16 moves reached their limit
  resources: environmentObjects/sektion_cabinet/ (28 files)
  class:     ihmc-high-level-behaviors/src/libgdx/java/us/ihmc/rdx/simulation/environment/object/objects/RDXSektionCabinetObject.java
  registered RDXSektionCabinetObject.FACTORY
```

Read the table before using the object. Masses come from the collision hulls at `--density`, and `drawer_bottom` is 43 kg because its collision is one solid box. Set a realistic mass, and pass `--force` because the object already exists:

```bash
python3 scripts/add_rdx_articulated_object_from_urdf.py ~/mujoco_assets/sektion_cabinet/sektion_cabinet.urdf \
  --part-mass drawer_bottom=3 --force
```

Despite its name, `drawer_bottom` is the drawer just above the doors.

### 3. Place it in RDX

Build and start the simulation UI (`AlexRDXSimulationUI` for Alex), then in the **Environment** panel check **Enable Environment Building** and place **Sektion Cabinet**. The root link's origin is the cabinet's center, so raise it until it rests on the floor (z = 0.385 m for this cabinet). The drawers open along the cabinet's +x.

The simulation compiles the cabinet into MuJoCo, and the log says `... and N articulated object(s)`. With the cabinet selected, the panel shows one slider per joint. Check **Override joint positions** to set them by hand. Uncheck it to let the simulation drive them again.

To start an environment with it, add it to that environment's JSON like any other object:

```json
{ "type": "RDXSektionCabinetObject", "x": 1.3, "y": -1.6, "z": 0.385, "qx": 0.0, "qy": 0.0, "qz": 1.0, "qs": 0.0 }
```

## How the object is simulated

`RDXURDFArticulatedObject` implements `RDXMujocoArticulatedObject`, like the articulated door. `RDXMujocoObjectCollisionSync` adds it to the MuJoCo world:

* The root part is welded to the world at the object's pose. Moving the object moves that weld.
* Every other part is a body with a `hinge` or `slide` joint, the URDF's axis and limits, its damping (2 when the URDF gives none), and `armature` 0.01.
* Every collision piece is a mesh geom in the `terrain` class. The robot and free objects collide with every part. Two `terrain` geoms never collide, so the parts of one object never collide with each other.
* Joint positions go back to RDX every frame, so the visuals follow the simulation.

The pairs in `<stem>_rdx.json` are written as MuJoCo `<exclude>` contacts:

* Each part and its parent across a movable joint. PhysX skips these contacts. MuJoCo only skips parent-child contacts when the parent is not welded to the world.
* With MuJoCo installed, any two unjointed parts that overlap by at most `--overlap-tolerance` (3 mm) in any combination of joint limits. That is an error in the source asset. The cabinet's right door overlaps the drawer above it by 0.5 mm when both are open. A deeper overlap is reported and kept, because it is more likely a real constraint, such as a drawer behind a closed door.

Then the script drives each joint to both limits, with the other joints closed and then open, and reports any move that stops short. A blocked move means the collision pieces still need work.

## URDF requirements

* Joints are `fixed`, `revolute`, `continuous` or `prismatic`. The links form one tree.
* Visuals and collisions are meshes, or `box`, `cylinder` and `sphere` primitives, which become meshes. Mesh paths are relative to the URDF, absolute, or `package://`. Give each package's folder with `--package-dir PKG=PATH`.
* Links without `<inertial>` get one from their collision hulls at `--density`. The root needs a visual.

## Options

| Option | Effect |
| --- | --- |
| `--name` | Display name in the environment builder. Also sets the class name, so `"Kitchen Cabinet"` becomes `RDXKitchenCabinetObject`. |
| `--stem` | Resource folder and URDF name. Defaults to the URDF file name. |
| `--density` | kg/m³ for links without `<inertial>`. Default is `1000`, as in Isaac Sim. |
| `--part-mass PART=KG` | Set one part's mass, scaling its inertia. Repeatable. Part names are in the table the script prints. |
| `--package-dir PKG=PATH` | Where `package://PKG/` resolves. Repeatable. |
| `--no-convex` | Keep collision surfaces instead of their convex hulls. |
| `--no-check` | Skip the MuJoCo overlap and joint-travel checks. |
| `--overlap-tolerance MM` | Deepest overlap excluded as an asset error. Default is `3`. |
| `--force` | Replace an existing resource folder and class. |
| `--dry-run` | Print the parts, joints and checks without writing files. |

# Export an RDX environment to Isaac Sim

The other direction: an RDX environment becomes one USD stage that Isaac Sim opens. USD is Isaac Sim's own format and holds both what a GLB holds (meshes, materials) and what a URDF holds (links, joints, limits, masses, colliders). No script is needed, because RDX exports from its loaded models, so `.g3dj`, `.glb` and procedural terrain all export the same way.

In the **Environment** panel, load or build the environment and click **Export to Isaac Sim**. The stage is written to `~/.ihmc/isaacSimExports/<environment>/<environment>.usda`, with its textures in `textures/` beside it. Open it in Isaac Sim with **File > Open** and press **Play**.

The stage has:

| RDX | USD |
| --- | --- |
| Each object's visual | `Mesh` prims under `<object>/Visuals`, with `UsdPreviewSurface` materials. Textures are read back from the GPU as PNGs. |
| The collision MuJoCo simulates (convex hull, box, sphere, procedural terrain pieces) | Hidden collider prims under `<object>/Collisions`. Hulls use the `convexHull` approximation. |
| Objects in `RDXInteractableObjectLibrary` (bottle, box, drill, ...) | Rigid bodies with the object's mass, so they fall and can be pushed. Everything else is static. |
| URDF objects and the articulated door | Fixed-base articulations: a rigid body per link, a revolute, prismatic or fixed joint per link with the same axis, limits, damping and spring as in MuJoCo, and the current joint positions. Contact exclusions become filtered pairs. |
| Point and directional lights | `SphereLight` and `DistantLight`, plus a `DomeLight` scaled by the ambient light. |
| MuJoCo's flat ground | An invisible ground plane at z = 0, unless the environment has procedural ground. |

The robot is not exported. Joint names are the URDF's, so the cabinet's joints are `door_left_joint`, `door_right_joint`, `drawer_bottom_joint` and `drawer_top_joint`. The door's latch is MuJoCo-side logic and is not exported. Its panel swings freely once the lever turns, or without it.

For example, EmptyRoom plus a cabinet, a bottle, a box, a table and uneven tiles exports in about a second as a 7 MB stage. In Isaac Sim the bottle dropped from 1 m to rest at z = 0.079 m, the same height as in MuJoCo. Pushing the cabinet's joints opened the top drawer to its 0.4 m limit and swung both doors open. The bottom drawer stayed behind the closed doors.

# Requirements

Python 3 with `trimesh` and `numpy`. `make_convex_collision_parts.py` also needs `scipy`:

```bash
python3 -c "import trimesh, numpy, scipy"
```

`add_rdx_articulated_object_from_urdf.py` also uses `mujoco`, if installed, for the overlap and joint-travel checks. Without it, the script prints a warning and excludes only jointed pairs:

```bash
pip install mujoco
python3 -c "import mujoco"
```
