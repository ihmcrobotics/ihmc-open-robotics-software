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
from llm_interface import LLMInterface
import re
import sys



# Example usage:
print(" Calling the LLM ")
llm = LLMInterface(config_file="config.json")

ros2 = {}
initialized = False
waiting_for_command = True

# Store the list of behaviors to execute in sequence
behavior_queue = []
current_behavior_index = 0

# Set counter to count the number of times LLM is called
llm_call_counter = 0

def behavior_message_callback(msg):
    #print("Received AI2R Status Message")
    global initialized  # Access the global variables
    global waiting_for_command
    global behavior_queue
    global current_behavior_index
    global llm_call_counter

    robot_pose = msg.robot_mid_feet_under_pelvis_pose_in_world

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
        else:
           print("-")
        scene_objects_name = [obj.object_name for obj in msg.objects]

        # --------- Behaviors -----------
        behaviors = msg.available_behaviors
  
        # Get available behaviors
        available_behaviors = msg.available_behaviors

        # Completed and failed behaviors
        completed_behavior = msg.completed_behavior
        failed_behavior = msg.failed_behavior
        #initialized = True

        #print("current_behavior_index: ", current_behavior_index)

        if completed_behavior and completed_behavior != "-":
            print(f"Completed Behavior: {completed_behavior}")
            waiting_for_command = True

        if failed_behavior:
            print(f"[FAILURE] Failed Behavior: {failed_behavior}")

        # Construct input for LLM decision-making
        llm_input = {
            # "task": current_task,
            "scene_objects": scene_objects_name,
            "available_behaviors": available_behaviors,
            "previously_executed": completed_behavior if completed_behavior != "-" else "",
            "failed_behaviors": failed_behavior if failed_behavior else "",
        }

        if waiting_for_command:
            # Convert the input to a string for LLM processing
            llm_input = str(llm_input)

            print("LLM Input for reasoning:", llm_input)

            # Check if it is first time calling the LLM
            if llm_call_counter == 0:
                llm.first_log_interaction(llm_input)
            # Query LLM for next action
            response        = llm.call_model(llm_input)
            
            # Increment the LLM call counter
            llm_call_counter += 1
            
            #print("LLM Response:", response)

            next_behavior   = response.strip()
            # # Extract content after the last </think>
            # match = re.search(r'</think>(.*)$', response, re.DOTALL)
            # if match:
            #     next_behavior = match.group(1).strip()
            # else:
            #     next_behavior = response.strip()            

        next_behavior = available_behaviors[current_behavior_index] if current_behavior_index < len(available_behaviors) else None
        

        # Check if the LLM suggests a valid action
        if waiting_for_command:
            if (next_behavior in available_behaviors):
                print("LLM Response:", next_behavior)
                # Execute the suggested behavior
                behavior_command = AI2RCommandMessage()
                behavior_command.behavior_to_execute = next_behavior
                print(f"Commanded Behavior: {next_behavior}")
                ros2["behavior_publisher"].publish(behavior_command)
                waiting_for_command = False
                current_behavior_index += 1  # Move to the next behavior in the queue

            else:
                sys.exit(1)
                print("[WARNING] LLM suggested an invalid action or no action needed.")



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