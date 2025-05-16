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
from cam_2_base_transform import *
import os
import glob

class PoseEstimationNode(Node):
    def __init__(self, mesh_file):
        super().__init__('pose_estimation_node')

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.cam_K = None
        self.mask = None  # Received from YOLO

        self.pose_estimator = None  # Will hold FoundationPose instance
        self.initialized = False

        # Load mesh and prepare bounding box
        # Load mesh and prepare bounding box
        mesh = trimesh.load(mesh_file)
        self.mesh = mesh  
        _, extents = trimesh.bounds.oriented_bounds(mesh)
        self.bbox = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)


        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        self.glctx = dr.RasterizeCudaContext()

        # ROS subscriptions
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/yolo/mask', self.mask_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, "/pose", 10)

    def camera_info_callback(self, msg):
        if self.cam_K is None:
            self.cam_K = np.array(msg.k).reshape((3, 3))

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1") / 1e3
        self.process()

    def mask_callback(self, msg):
        self.mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def process(self):
        if None in (self.color_image, self.depth_image, self.cam_K, self.mask):
            return

        if not self.initialized:
            self.pose_estimator = FoundationPose(
                model_pts=self.mesh.vertices,
                model_normals=self.mesh.vertex_normals,
                mesh=self.mesh,
                scorer=self.scorer,
                refiner=self.refiner,
                glctx=self.glctx
            )
            pose = self.pose_estimator.register(
                K=self.cam_K, rgb=self.color_image, depth=self.depth_image, ob_mask=self.mask, iteration=4
            )
            self.initialized = True
        else:
            pose = self.pose_estimator.track_one(
                rgb=self.color_image, depth=self.depth_image, K=self.cam_K, iteration=2
            )

        center_pose = pose  # Replace with transformed pose if needed
        self.publish_pose(center_pose)

        vis = draw_posed_3d_box(self.cam_K, self.color_image.copy(), center_pose, self.bbox)
        vis = draw_xyz_axis(vis, center_pose, scale=0.1, K=self.cam_K, thickness=2, transparency=0, is_input_rgb=True)
        cv2.imshow("Pose Tracking", vis[..., ::-1])
        cv2.waitKey(1)

    def publish_pose(self, T):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_link"  # or whatever you use

        t = T[:3, 3]
        q = R.from_matrix(T[:3, :3]).as_quat()

        pose_msg.pose.position.x = t[0]
        pose_msg.pose.position.y = t[1]
        pose_msg.pose.position.z = t[2]
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    # Use the first .obj file in demo_data
    base_dir = os.path.dirname(os.path.realpath(__file__))
    demo_obj = glob.glob(os.path.join(base_dir, "demo_data", "**", "*.obj"), recursive=True)[0]

    node = PoseEstimationNode(demo_obj)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()