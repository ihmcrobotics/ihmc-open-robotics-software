import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from typing import List, Optional
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from behavior_msgs.msg import AI2RCommandMessage
from behavior_msgs.msg import AI2RObjectMessage
from behavior_msgs.msg import AI2RStatusMessage
from behavior_msgs.msg import AI2RNavigationMessage
from behavior_msgs.msg import AI2RHandPoseAdaptationMessage

import cv2
import numpy as np
from llm_interface import LLMInterface
import re
import sys

# Example usage:
print(" Calling the LLM ")
llm = LLMInterface(config_file="config_goto.json")

ros2 = {}
initialized = False
waiting_for_command = True

# Set counter to count the number of times LLM is called
llm_call_counter = 0

def behavior_message_callback(msg):
    #print("Received AI2R Status Message")
    global initialized  # Access the global variables
    global waiting_for_command
    global llm_call_counter

    if not initialized:
        # --------- Scene -----------
        scene_objects = msg.objects
        #print("Objects in the scene:")
        #print("scene_objects: ",scene_objects)
        if scene_objects:  # This checks if the list is not empty
           for obj in scene_objects:
               id = obj.object_name
               #print(f"{id}")
               pose_in_world = obj.object_pose_in_world
               pose_wrt_robot = obj.object_pose_in_robot_frame # This is the pose specified wrt to robot_pose
#         else:
#            print("-")
        scene_objects_names = [obj.object_name for obj in msg.objects]
        scene_objects_poses = [obj.object_pose_in_world for obj in msg.objects]
        scene_objects_positions = [pose.position for pose in scene_objects_poses]
        robot_position = msg.robot_mid_feet_under_pelvis_pose_in_world.position

        # Completed and failed behaviors
        behavior_in_progress = msg.behavior_in_progress

        waiting_for_command = False
        if msg.behavior_in_progress == "-":
            waiting_for_command  = True

        # Construct input for LLM decision-making
        llm_input = {
            # "task": current_task,
            "scene_objects_names": scene_objects_names,
        }

        if waiting_for_command:
            # Convert the input to a string for LLM processing
            llm_input = str(llm_input)

            print("LLM Input for reasoning:", llm_input)

            # Check if it is first time calling the LLM
            if llm_call_counter == 0:
                llm.first_log_interaction(llm_input)
            # Query LLM for next action
            response  = llm.call_model(llm_input)
            print("LLM Response:", response)

            # Split by line breaks
            lines = response.strip().split('\n')
            # Each variable now holds one line
            base_name = lines[0]
            spatial_context_object = lines[1]
            spatial_relation = lines[2]




            # Increment the LLM call counter
            llm_call_counter += 1


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
