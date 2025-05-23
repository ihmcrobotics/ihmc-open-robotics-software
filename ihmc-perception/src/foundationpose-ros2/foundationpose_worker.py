import numpy as np
import trimesh
from estimater import FoundationPose

class FoundationPoseWorker:
    def __init__(self, mesh, rgb, depth, mask, camera_k, object_id=None, glctx=None):
        self.foundation_pose = FoundationPose(
            model_pts=mesh.vertices,
            model_normals=mesh.vertex_normals,
            mesh=mesh,
            glctx=glctx
        )

        self.camera_k = camera_k

        self.initial_rgb = rgb
        self.initial_depth = depth
        self.initial_mask = mask
        self.object_id = object_id
        self.initialized = False

        to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
        self.to_origin = to_origin
        self.bbox = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)


    def update(self, rgb, depth):
        if not self.initialized:
            print("REGISTERING")
            self.foundation_pose.register(K=self.camera_k, rgb=self.initial_rgb, depth=self.initial_depth, ob_mask=self.initial_mask, ob_id=self.object_id)
            self.initialized = True

        print("TRACKING")
        pose = self.foundation_pose.track_one(rgb=rgb, depth=depth, K=self.camera_k, iteration=2)
        center_pose = pose@np.linalg.inv(self.to_origin)

        # TODO: Add bounding box to result
        return center_pose
