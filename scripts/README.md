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

Python 3 with `trimesh` and `numpy`:

```bash
python3 -c "import trimesh, numpy"
```
