import sys
import os

# sys.path.append(os.path.join(os.path.dirname(__file__), 'FoundationPose'))
base_dir = os.path.dirname(os.path.abspath(__file__))  # Just get the current script folder
sys.path.append(os.path.join(base_dir, 'FoundationPose'))
sys.path.append(os.path.join(base_dir, 'FoundationPose', 'nvdiffrast'))


import rclpy
from rclpy.node import Node
from estimater import *
import cv2
import numpy as np
import trimesh
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge
import argparse
import os
from scipy.spatial.transform import Rotation as R
from ultralytics import SAM
from cam_2_base_transform import *
import glob


class FoundationPoseWorker
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


    def update(self, rgb, depth):
        if not self.initialized:
            self.foundation_pose.register(K=self.camera_k, rgb=self.initial_rgb, depth=self.initial_depth, ob_mask=self.initial_mask, ob_id=object_id)
            self.initialized = True

        return self.foundation_pose.track_one(rgb=rgb, depth=depth, K=camera_k, iteration=2)
