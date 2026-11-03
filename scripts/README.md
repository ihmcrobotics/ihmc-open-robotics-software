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

## Requirements

Python 3 with `trimesh` and `numpy`. `make_convex_collision_parts.py` also needs `scipy`:

```bash
python3 -c "import trimesh, numpy, scipy"
```
