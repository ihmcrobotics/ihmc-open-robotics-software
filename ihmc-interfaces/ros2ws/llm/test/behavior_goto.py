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

# Call the LLMInterface class
llm                 = LLMInterface(config_file="config.json")

ros2                = {}
initialized         = False
waiting_for_command = True
llm_call_counter    = 0

def behavior_message_callback(msg):
    global initialized  # Access the global variables
    global waiting_for_command
    global llm_call_counter
    robot_pose = msg.robot_mid_feet_under_pelvis_pose_in_world

    #print("initialized: ", initialized)
    #print("waiting_for_command: ", waiting_for_command)
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


    if not waiting_for_command and initialized:
        # --------- Monitoring -----------
        completed_behavior = msg.completed_behavior

        if not completed_behavior == "-":
            print("Completed Behavior: " + completed_behavior)
            waiting_for_command  = True
        else:
            waiting_for_command  = False

        failed_behavior = msg.failed_behavior
        if not failed_behavior == "-":
            print("[FAILURE] Failed behavior: " + failed_behavior)
            #Failure details
            failure = msg.failure
            print("Description: " + failure.action_name)
            print("Type: " + failure.action_type)
            print("Frame: " + failure.action_frame)

            position_error = failure.position_error
            # Convert Point to numpy array
            error_vector = np.array([position_error.x, position_error.y, position_error.z])
            # Calculate the Euclidean norm (L2 norm)
            norm = np.linalg.norm(error_vector)
            print(f"The position error is: {norm}")
            orientation_error = failure.orientation_error

            position_tolerance = failure.position_tolerance
            orientation_tolerance = failure.orientation_tolerance



    if waiting_for_command or not initialized:
        # --------- Reasoning -----------
        # CAN DO SOME REASONING HERE based on failed behaviors
        
        # Get all scene objects names
        scene_objects_name  = []
        scene_objects = msg.objects
        if scene_objects:  # This checks if the list is not empty
           for obj in scene_objects:
               id = obj.object_name
               pose_in_world = obj.object_pose_in_world
               pose_wrt_robot = obj.object_pose_in_robot_frame # This is the pose specified wrt to robot_pose
               #print(f"{id} - Pose in World: {pose_in_world}, Pose wrt Robot: {pose_wrt_robot}")
               #insert id, pose_in_world, pose_wrt_robot into a dictionary or list if needed
               object_list = {
                   "object_name": id,
                   "pose_in_world": {
                       "pose" : pose_in_world.position,
                       "orientation": pose_in_world.orientation
                    #    "x": pose_in_world.position.x,
                    #    "y": pose_in_world.position.y,
                    #    "z": pose_in_world.position.z
                   },
                   "pose_wrt_robot": {
                        "pose": pose_wrt_robot.position,
                        "orientation": pose_wrt_robot.orientation
                    #    "x": pose_wrt_robot.position.x,
                    #    "y": pose_wrt_robot.position.y,
                    #    "z": pose_wrt_robot.position.z
                   }
               }
               scene_objects_name.append(object_list)
        #print("Scene Objects:", scene_objects_name)

        
        # Get all available behaviors
        available_behaviors = msg.available_behaviors
        completed_behavior  = msg.completed_behavior
        failed_behavior     = msg.failed_behavior
        
        # Construct input for LLM decision-making
        llm_input = {
            # "task": current_task,
            "scene_objects": scene_objects_name,
            "available_behaviors": available_behaviors,
            "previously_executed": completed_behavior if completed_behavior != "-" else "",
            "failed_behaviors": failed_behavior if failed_behavior else "",
        }
        
        #if waiting_for_command :
        # Convert the input to a string for LLM processing
        llm_input       = str(llm_input)

        print("LLM Input for reasoning:", llm_input)

        # Check if it is first time calling the LLM
        if llm_call_counter == 0:
            llm.first_log_interaction(llm_input)
            
        # Query LLM for next action
        print(" Calling the LLM ")
        response        = llm.call_model(llm_input)
        # if llm_call_counter == 0:
        #     response         =  "GOTO CHARGE" # For testing purposes
        # elif llm_call_counter == 1:
        #     response         =  "SCAN"
        # elif llm_call_counter == 2:
        #     response         =  "PICK UP CHARGE"
        # elif llm_call_counter == 3:
        #     response         =  "GOTO DOOR"
        # elif llm_call_counter == 4:
        #     response         =  "PLACE CHARGE ON DOOR"
        # else:
        #     # end the program after 5 calls
        #     print("Reached maximum number of LLM calls. Exiting.")
        #     sys.exit(0)
        

        # Increment the LLM call counter
        llm_call_counter += 1
        
        print("LLM Response:", response, "type of response:", type(response), "length of response:", len(response))
        print("LLM Call Counter:", llm_call_counter)

        next_behavior   = response.strip('.\"')  # Remove any trailing punctuation or quotes
        print("Next behavior:", next_behavior, "type of next behavior:", type(next_behavior), "length of next behavior:", len(next_behavior))

        # --------- Coordination -----------
        behavior_command = AI2RCommandMessage()
        # DECIDE what behavior to execute based on reasoning. For example can decide to scan environment to detect objects
        #behavior_command.behavior_to_execute = "GOTO CHARGE"
        behavior_command.behavior_to_execute = next_behavior
        print("Commanded Behavior: " + behavior_command.behavior_to_execute)
        ros2["behavior_publisher"].publish(behavior_command)
        waiting_for_command = False
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
