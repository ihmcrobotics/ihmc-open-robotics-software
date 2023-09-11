import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from lib.opts import opts
from lib.detectors.object_pose import ObjectPoseDetector

from geometry_msgs.msg import PoseStamped
from perception_msgs.msg import ImageMessage

import cv2
import time

class Image2CenterPose_node(Node):
    def __init__(self):
        super().__init__('zed2_centerpose_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Default params with commandline input
        self.opt = opts().parser.parse_args()

        self.opt.arch = 'dlav1_34'
        self.opt.load_model = f"/root/centerpose-ros2/models/chair_v1_140.pth"
        self.opt.debug = 5

        # Default setting
        self.opt.nms = True
        self.opt.obj_scale = True
        self.image_process_frequency = 10
        self.image_process_period_ns = int(1e9 / self.image_process_frequency)
        self.last_image_process_time_ns = 0

        # Tracking stuff
        if self.opt.tracking_task == True:
            print('Running tracking')
            self.opt.pre_img = True
            self.opt.pre_hm = True
            self.opt.tracking = True
            self.opt.pre_hm_hp = True
            self.opt.tracking_hp = True
            self.opt.track_thresh = 0.1

            self.opt.obj_scale_uncertainty = True
            self.opt.hps_uncertainty = True
            self.opt.kalman = True
            self.opt.scale_pool = True

            self.opt.vis_thresh = max(self.opt.track_thresh, self.opt.vis_thresh)
            self.opt.pre_thresh = max(self.opt.track_thresh, self.opt.pre_thresh)
            self.opt.new_thresh = max(self.opt.track_thresh, self.opt.new_thresh)

            # # For tracking moving objects, better to set up a small threshold
            # opt.max_age = 2

            print('Using tracking threshold for out threshold!', self.opt.track_thresh)

        # PnP related
        self.meta = {}
        if self.opt.cam_intrinsic is None:
            self.meta['camera_matrix'] = np.array(
                [[663.0287679036459, 0, 300.2775065104167], [0, 663.0287679036459, 395.00066121419275], [0, 0, 1]])
            self.opt.cam_intrinsic = self.meta['camera_matrix']
        else:
            self.meta['camera_matrix'] = np.array(self.opt.cam_intrinsic).reshape(3, 3)

        self.opt.use_pnp = True

        # Update default configurations
        self.opt = opts().parse(self.opt)

        # Update dataset info/training params
        self.opt = opts().init(self.opt)

        if self.opt.use_pnp == True and 'camera_matrix' not in self.meta.keys():
            raise RuntimeError('Error found. Please give the camera matrix when using pnp algorithm!')

        os.environ['CUDA_VISIBLE_DEVICES'] = self.opt.gpus_str
        self.opt.debug = max(self.opt.debug, 1)
        # Detector = detector_factory[self.opt.task]
        # self.detector = Detector(self.opt)
        self.detector = ObjectPoseDetector(self.opt)
        self.detector.pause = False

        self.subscription = self.create_subscription(
            ImageMessage,
            '/ihmc/zed2/left_color',
            self.listener_callback,
            qos_profile)
        self.subscription

        # Create a publisher for the pose topic
        self.pose_pub = self.create_publisher(PoseStamped, '/pose_topic', 10)
        self.pose_msg = PoseStamped()

        self.get_logger().info("Waiting for an Image...")

    def listener_callback(self, msg):
        # Skip the ImageMessage if not enough time has passed since we processed the last one to save CPU - it can't
        # keep up at 30hz even on an i7 13700 -danderson
        if time.time_ns() - self.last_image_process_time_ns < self.image_process_period_ns:
            return
        self.last_image_process_time_ns = time.time_ns()

        self.get_logger().info("Processing ImageMessage #" + str(msg.sequence_number))
        image_np = np.frombuffer(b''.join(msg.data), dtype=np.uint8)
        image = cv2.imdecode(image_np, cv2.COLOR_YUV2RGB)
        ret = self.detector.run(np.asarray(image), meta_inp=self.meta)

        if len(ret['results']) > 0:
            # print(ret)
            self.pose_msg.pose.position.x = ret['results'][0]['location'][0]
            self.pose_msg.pose.position.y = ret['results'][0]['location'][1]
            self.pose_msg.pose.position.z = ret['results'][0]['location'][2]
            self.pose_msg.pose.orientation.x = ret['results'][0]['quaternion_xyzw'][0]
            self.pose_msg.pose.orientation.y = ret['results'][0]['quaternion_xyzw'][1]
            self.pose_msg.pose.orientation.z = ret['results'][0]['quaternion_xyzw'][2]
            self.pose_msg.pose.orientation.w = ret['results'][0]['quaternion_xyzw'][3]
            self.get_logger().info('Published: "%s"' % self.pose_msg.pose)
            self.pose_pub.publish(self.pose_msg)

def main(args=None):
    # Check if models directory exists
    # Download pretrained models from here https://drive.google.com/drive/folders/16HbCnUlCaPcTg4opHP_wQNPsWouUlVZe
    if not os.path.exists('models'):
        print('Could not find models directory')
        return

    rclpy.init(args=args)

    node = Image2CenterPose_node()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()