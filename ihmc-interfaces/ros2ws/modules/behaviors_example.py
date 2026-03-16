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

import cv2
import numpy as np

ros2 = {}
first_command = True

def behavior_message_callback(msg):
   global first_command
   robot_pose = msg.robot_mid_feet_under_pelvis_pose_in_world

   if first_command:
       # --------- Behaviors -----------
       behaviors = msg.available_behaviors
       print("Available behaviors:")
       if behaviors:
            for behavior in behaviors:
                print(behavior)
       else:
            print("-")

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

   # --------- Monitoring -----------
   completed_behavior = msg.completed_behavior
   failed_behavior = msg.failed_behavior
   if failed_behavior != "-":
       print("[FAILURE] Failed behavior: " + failed_behavior)
       # Failure details
       failure = msg.failure
       print("Description: " + failure.action_name)
       print("Type: " + failure.action_type)
       print("Frame: " + failure.action_frame)
       print("Missing Frame: " + str(failure.missing_frame))
       print("Navigation Collision Frame Name: " + failure.collision_name)

       position_error = failure.position_error
       # Convert Point to numpy array
       error_vector = np.array([position_error.x, position_error.y, position_error.z])
       # Calculate the Euclidean norm (L2 norm)
       norm = np.linalg.norm(error_vector)
       print(f"The position error is: {norm}")
       orientation_error = failure.orientation_error

       position_tolerance = failure.position_tolerance
       orientation_tolerance = failure.orientation_tolerance

   # --------- Coordination / Adaptation -----------
   if completed_behavior != "-" or first_command:
       print("Completed Behavior: " + completed_behavior)
       first_command = False
       behavior_command = AI2RCommandMessage()
       # DECIDE what behavior to execute based on reasoning.
       behavior_command.behavior_to_execute = "GOTO"

       # Update the go to behavior to navigate to whatever object or whenever in space according to reasoning
       new_goto_behavior = AI2RNavigationMessage()
       # Set the reference frame name - can copy from scene_objects.obj_name
       new_goto_behavior.target_object = "Walking"
       # Set the distance to the object
       new_goto_behavior.distance_to_object = 0.6
       behavior_command.navigation = new_goto_behavior
       print("Commanding behavior: " + behavior_command.behavior_to_execute)

       ros2["behavior_publisher"].publish(behavior_command)


def main(args=None):
    rclpy.init(args=args)
    
    qos_profile_reliable = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
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
