import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from behavior_msgs.msg import AI2RCommandMessage
from behavior_msgs.msg import AI2RObjectMessage
from behavior_msgs.msg import AI2RStatusMessage
from behavior_msgs.msg import AI2RNavigationMessage
from behavior_msgs.msg import AI2RHandPoseAdaptationMessage

import cv2
import numpy as np

ros2 = {}
initialized = False

def behavior_message_callback(msg):
    global initialized  # Access the global variables
    robot_pose = msg.robot_mid_feet_under_pelvis_pose_in_world

    if not initialized:
        # --------- Scene -----------
        scene_objects = msg.objects
        print("Objects in the scene:")
        if scene_objects:  # This checks if the list is not empty
           for obj in scene_objects:
               id = obj.object_name
               print(f"{id}")
               pose_in_world = obj.object_pose_in_world
               pose_wrt_robot = obj.object_pose_in_robot_frame # This is the pose specified wrt to robot_pose
        else:
           print("-")

        # --------- Behaviors -----------
        behaviors = msg.available_behaviors
        print("Available behaviors:")
        if behaviors:
            for behavior in behaviors:
                print(behavior)
        else:
            print("-")

    # --------- Reasoning -----------
    # CAN DO SOME REASONING HERE based on objects in the scene and available behaviors

    # --------- Monitoring -----------
    print("Behavior in Progress: " + msg.behavior_in_progress)
    print("Completed Behavior: " + msg.completed_behavior)

    failed_behavior = msg.failure
    if failed_behavior:
        print("[FAILURE] -----------")
        print("Failed behavior: ")
        # Failure details
        print("Name: " + failed_behavior.action_name)
        print("Type: " + failed_behavior.action_type)
#         print("Frame: " + failed_behavior.action_frame)
        print("Missing Frame: " + str(failed_behavior.missing_frame))
        print("Navigation Collision Frame Name: " + failed_behavior.collision_name)

        position_error = failed_behavior.position_error
        # Convert Point to numpy array
        error_vector = np.array([position_error.x, position_error.y, position_error.z])
        # Calculate the Euclidean norm (L2 norm)
        norm = np.linalg.norm(error_vector)
        print(f"The position error is: {norm}")
        orientation_error = failed_behavior.orientation_error

        position_tolerance = failed_behavior.position_tolerance
        orientation_tolerance = failed_behavior.orientation_tolerance
        print("----------[FAILURE]")

    # --------- Reasoning -----------
    # CAN DO SOME REASONING HERE based on failed behaviors
    waiting_for_command = False
    if msg.behavior_in_progress == "-":
       waiting_for_command  = True

    if waiting_for_command or not initialized:
        # --------- Coordination -----------
        behavior_command = AI2RCommandMessage()
        # DECIDE what behavior to execute based on reasoning. For example can decide to scan environment to detect objects
        behavior_command.behavior_to_execute = "GOTO"
        behavior_command.adapting_behavior = True

        new_goto_behavior = AI2RNavigationMessage()
        # Set the reference frame name - can copy from scene_objects.obj_name
        new_goto_behavior.object_name = "Barrier1"
        # Set the distance to the object
        new_goto_behavior.distance_to_object = 0.6
        new_goto_behavior.pov_reference_frame_name = "DoorPanel1"
        new_goto_behavior.spatial_relation = AI2RNavigationMessage.FRONT
        if new_goto_behavior.spatial_relation == AI2RNavigationMessage.DEFAULT:
            new_goto_behavior.pov_reference_frame_name = "walkingFrame"

        behavior_command.navigation = new_goto_behavior

        print("Commanded Behavior: " + behavior_command.behavior_to_execute)
        ros2["behavior_publisher"].publish(behavior_command)
        initialized = True


def main(args=None):
    rclpy.init(args=args)
    
    qos_profile_reliable = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )

    node = rclpy.create_node('behavior_coordination_node')
    ros2["node"] = node

    behavior_subscriber = node.create_subscription(AI2RStatusMessage,
                                            '/ihmc/behavior_tree/ai2r_status',
                                            behavior_message_callback, qos_profile_reliable)
    ros2["behavior_subscriber"] = behavior_subscriber

    behavior_publisher = node.create_publisher(AI2RCommandMessage,
                                    '/ihmc/behavior_tree/ai2r_command',
                                    qos_profile_reliable)
    ros2["behavior_publisher"] = behavior_publisher

    # Wait until interrupt
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == '__main__':
    main()
