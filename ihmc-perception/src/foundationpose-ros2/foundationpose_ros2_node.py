import sys
import os

base_dir = os.path.dirname(os.path.abspath(__file__))  # Just get the current script folder
sys.path.append(os.path.join(base_dir, 'FoundationPose'))
sys.path.append(os.path.join(base_dir, 'FoundationPose', 'nvdiffrast'))

import rclpy
from rclpy.node import Node
from estimater import *
import cv2
from cv_bridge import CvBridge
import trimesh
import glob
import numpy as np
import threading
from sensor_msgs.msg import Image, CameraInfo
from perception_msgs.msg import FoundationPoseRequest
from foundationpose_worker import FoundationPoseWorker

COLOR_TOPIC = '/foundation_pose/camera/color/image_raw'
DEPTH_TOPIC = '/foundation_pose/camera/aligned_depth_to_color/image_raw'
CAMERA_INFO_TOPIC = '/foundation_pose/camera/color/camera_info'
REQUEST_TOPIC = '/foundation_pose/request'
RESULT_TOPIC = '/foundation_pose/result'

class FoundationPoseROS2Node(Node):
    def __init__(self, mesh_file_paths):
        super().__init__('foundation_pose_node')

        print("Rasterizing CUDA context...")
        self.glctx = dr.RasterizeCudaContext()

        print("Reading meshes...")
        self.meshes = {os.path.basename(file_path): trimesh.load(file_path) for file_path in mesh_file_paths}
        self.workers = {}

        self.bridge = CvBridge()

        self.rgb = None
        self.depth = None
        self.camera_k = None

        self.new_depth_available = threading.Event()
        self.new_color_available = threading.Event()

        # Create subscriptions to camera topics (color, depth, camera info)
        print("Creating subscriptions...")
        self.rgb_subscription = self.create_subscription(Image, COLOR_TOPIC, self.color_callback, 10)
        self.depth_subscription = self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.camera_info_subscription = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)

        # Create subscription to request messages
        self.request_subscription = self.create_subscription(FoundationPoseRequest, REQUEST_TOPIC, self.request_callback, 10)

        print("Starting the process thread...")
        self.pose_estimation_thread = threading.Thread(target=self.process)
        self.pose_estimation_thread.start()


    def color_callback(self, color_image):
        self.rgb = self.bridge.imgmsg_to_cv2(color_image, "rgb8")
        self.new_color_available.set()


    def depth_callback(self, depth_image):
        self.depth = self.bridge.imgmsg_to_cv2(depth_image, "32FC1") / 1e3
        self.new_depth_available.set()


    def camera_info_callback(self, camera_info):
        self.camera_k = np.array(camera_info.k).reshape((3, 3))


    def request_callback(self, request):
        print("REQUEST RECEIVED")
        if self.camera_k is None:
            print("idk the camera intrinsics. Not accepting request atm")
            return

        mesh = self.meshes[request.mesh_file]
        color = self.bridge.imgmsg_to_cv2(request.color, "rgb8")
        depth = self.bridge.imgmsg_to_cv2(request.depth, "32FC1") / 1e3
        mask = self.bridge.imgmsg_to_cv2(request.object_mask, "8UC1")

        if mask.shape[:2] != color.shape[:2]:
            height, width = color.shape[:2]
            mask = cv2.resize(mask, (width, height))

        workers[request.object_id] = FoundationPoseWorker(
            mesh=mesh,
            rgb=color,
            depth=depth,
            camera_k=self.camera_k,
            object_id=request.object_id,
            glctx=self.glctx
        )

    def process(self):
        print("Starting process thread...")
        try:
            while True:
                # Wait for new images to be available
                self.new_color_available.wait()
                self.new_depth_available.wait()

                for _, worker in self.workers.items():
                    worker.update(self.rgb, self.depth)

                self.new_color_available.clear()
                self.new_depth_available.clear()
        except KeyboardInterrupt:
            print("Process finishing...")

def main():
    # initialize ROS
    rclpy.init()

    # Find mesh files paths
    base_directory = os.path.dirname(os.path.realpath(__file__))
    mesh_file_paths = glob.glob(os.path.join(base_dir, "demo_data", "**", "*.obj"), recursive=True)

    print("Found meshes", mesh_file_paths)

    # Create the FoundationPose node and spin it
    node = FoundationPoseROS2Node(mesh_file_paths=mesh_file_paths)
    print("Spinning the node...")
    rclpy.spin(node)

    # Destroy the node and shut down ROS
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
