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
import threading
import queue
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String as StringMessage
from perception_msgs.msg import FoundationPoseRequest, FoundationPoseResult
from foundationpose_worker import FoundationPoseWorker

COLOR_TOPIC = '/foundation_pose/camera/color/image_raw'
DEPTH_TOPIC = '/foundation_pose/camera/aligned_depth_to_color/image_raw'
CAMERA_INFO_TOPIC = '/foundation_pose/camera/color/camera_info'
REQUEST_TOPIC = '/foundation_pose/request'
REMOVE_TOPIC = '/foundation_pose/remove'
RESULT_TOPIC = '/foundation_pose/result'

IMAGE_SCALE = 0.5

IHMC_COORD_ROTATION = np.array([
    [ 0, 0, 1],
    [-1, 0, 0],
    [ 0,-1, 0]
])

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

        self.remove_queue = queue.Queue()

        self.new_depth_available = threading.Event()
        self.new_color_available = threading.Event()

        # Create subscriptions to camera topics (color, depth, camera info)
        print("Creating subscriptions...")
        self.rgb_subscription = self.create_subscription(Image, COLOR_TOPIC, self.color_callback, 10)
        self.depth_subscription = self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.camera_info_subscription = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)

        # Create subscription to request messages
        self.request_subscription = self.create_subscription(FoundationPoseRequest, REQUEST_TOPIC, self.request_callback, 10)

        # Create subscription to reset messages
        self.reset_subscription = self.create_subscription(StringMessage, REMOVE_TOPIC, self.remove_callback, 10)

        # Create a result publisher
        print("Creating publisher...")
        self.result_publisher = self.create_publisher(FoundationPoseResult, RESULT_TOPIC, 10)

        print("Starting the process thread...")
        self.pose_estimation_thread = threading.Thread(target=self.process, daemon=True)
        self.pose_estimation_thread.start()


    def color_callback(self, color_image):
        big_rgb = self.bridge.imgmsg_to_cv2(color_image, "rgb8")
        self.rgb = cv2.resize(src=big_rgb, dsize=None, fx=IMAGE_SCALE, fy=IMAGE_SCALE)
        self.new_color_available.set()


    def depth_callback(self, depth_image):
        big_depth = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
        self.depth = cv2.resize(src=big_depth, dsize=None, fx=IMAGE_SCALE, fy=IMAGE_SCALE) / 1e3
        self.new_depth_available.set()


    def camera_info_callback(self, camera_info):
        k = np.array(camera_info.k).reshape((3, 3))
        k[:2] *= IMAGE_SCALE
        self.camera_k = k


    def request_callback(self, request):
        print("REQUEST RECEIVED")
        if self.camera_k is None:
            print("Idk the camera intrinsics. Not accepting request atm")
            return

        if request.object_id in self.workers:
            print("Already tracking this obj")
            return

        if not request.mesh_file in self.meshes:
            print("Idk this mesh file. Haven't seen anything like it: ", request.mesh_file)
            return

        # Get the message data
        mesh = self.meshes[request.mesh_file]
        color = self.bridge.imgmsg_to_cv2(request.color, "rgb8")
        depth = self.bridge.imgmsg_to_cv2(request.depth, "32FC1")
        mask = self.bridge.imgmsg_to_cv2(request.object_mask, "8UC1")

        # Scale images
        color = cv2.resize(src=color, dsize=None, fx=IMAGE_SCALE, fy=IMAGE_SCALE)
        depth = cv2.resize(src=depth, dsize=None, fx=IMAGE_SCALE, fy=IMAGE_SCALE) / 1e3

        # Ensure mask is same size as color
        if mask.shape[:2] != color.shape[:2]:
            height, width = color.shape[:2]
            mask = cv2.resize(mask, (width, height))

        self.workers[request.object_id] = FoundationPoseWorker(
            mesh=mesh,
            rgb=color,
            depth=depth,
            mask=mask,
            camera_k=self.camera_k,
            object_id=request.object_id,
            glctx=self.glctx
        )


    def remove_callback(self, remove_target):
        if remove_target.data in self.workers:
            self.remove_queue.put(remove_target.data)


    def process(self):
        print("Starting process thread...")
        while True:
            # Wait for new images to be available
            self.new_color_available.wait()
            self.new_depth_available.wait()

            # Remove workers requested on the remove topic
            while not self.remove_queue.empty():
                remove_target = self.remove_queue.get()
                if remove_target in self.workers:
                    del self.workers[remove_target]

            # Update each worker and publish the result pose
            for object_id, worker in self.workers.items():
                pose = worker.update(self.rgb, self.depth)

                # Get the position and rotation (Z forward, Y up)
                position = pose[:3, 3]
                rotation_matrix = pose[:3, :3]

                # Transform to X forward Z up coordinates
                position = IHMC_COORD_ROTATION @ position
                rotation_matrix = IHMC_COORD_ROTATION @ rotation_matrix @ IHMC_COORD_ROTATION.T

                # Create the result message and publish it
                result = FoundationPoseResult()
                result.object_id = object_id
                result.object_pose.position.x = position[0]
                result.object_pose.position.y = position[1]
                result.object_pose.position.z = position[2]

                quaternion = R.from_matrix(rotation_matrix).as_quat()
                result.object_pose.orientation.x = quaternion[0]
                result.object_pose.orientation.y = quaternion[1]
                result.object_pose.orientation.z = quaternion[2]
                result.object_pose.orientation.w = quaternion[3]

                self.result_publisher.publish(result)

            self.new_color_available.clear()
            self.new_depth_available.clear()


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
