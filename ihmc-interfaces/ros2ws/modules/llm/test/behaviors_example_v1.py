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

# LLM API imports
from together import Together
import os
import requests
import json

# Get a free API key from https://api.together.xyz/settings/api-keys
os.environ["TOGETHER_API_KEY"] = ""
def llama32(messages, model_size=3):
  model = f"meta-llama/Llama-3.2-{model_size}B-Instruct-Turbo"
  url = "https://api.together.xyz/v1/chat/completions"
  payload = {
    "model": model,
    "max_tokens": 4096,
    "temperature": 0.0,
    "stop": ["<|eot_id|>","<|eom_id|>"],
    "messages": messages
  }

  headers = {
    "Accept": "application/json",
    "Content-Type": "application/json",
    "Authorization": "Bearer " + os.environ["TOGETHER_API_KEY"]
  }
  res = json.loads(requests.request("POST", url, headers=headers, data=json.dumps(payload)).content)

  if 'error' in res:
    raise Exception(res['error'])

  return res['choices'][0]['message']['content']


ros2 = {}

def behavior_message_callback(msg):
   print("Received AI2R Status Message")
   robot_pose = msg.robot_mid_feet_under_pelvis_pose_in_world

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

   # --------- Reasoning -----------
   # CAN DO SOME REASONING HERE based on objects in the scene and available behaviors
   behaviors_str = str(behaviors)
   messages = [
   {
       "role": "assistant",
       "content": behaviors_str
   },
   {
       "role": "user",
       "content": "Summarize all the behaviors in one paragraph"
   }
   ]
   answer = llama32(messages)
   print("Summarized behaviors: ",answer)

   # --------- Monitoring -----------
   completed_behavior = msg.completed_behavior
   print("Completed Behavior: " + completed_behavior)
   failed_behavior = msg.failed_behavior
   if failed_behavior:
       print("[FAILURE] Failed behavior: " + failure_behavior)
       # Failure details
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

   # --------- Reasoning -----------
   # CAN DO SOME REASONING HERE based on failed behaviors

   # --------- Coordination / Adaptation -----------
   behavior_command = AI2RCommandMessage()
   # DECIDE what behavior to execute based on reasoning. For example can decide to navigate to a specific object
   behavior_command.behavior_to_execute = "GOTO"

   # Update the go to behavior to navigate to whatever object or whenever in space according to reasoning
   new_goto_behavior = AI2RNavigationMessage()
   # Set the reference frame name - can copy from scene_objects.obj_name
   new_goto_behavior.reference_frame_name = "Charge1"
   # Set the goal stance point - where the robot stance is positioned (position only, no orientation) wrt to the reference_frame_name
   new_goto_behavior.goal_stance_point = Point(x=1.0, y=2.0, z=0.0)
   # Set the goal focal point - where the stance is facing (how it is oriented) wrt to the reference_frame_name
   new_goto_behavior.goal_focal_point = Point(x=3.0, y=4.0, z=0.0)
   behavior_command.navigation = new_goto_behavior

   # CAN EDIT HAND POSE ACTION, IF failed behavior has failed because of that action
   new_hand_pose_action = AI2RHandPoseAdaptationMessage()
   # Set the name of the action. COPY THAT from failed_behavior message
   new_hand_pose_action.action_name = "action_to_modify"
   # Set the reference frame name - can copy from scene_objects.obj_name
   new_hand_pose_action.reference_frame_name = "Barrier1"
   # Set the new position
   new_hand_pose_action.new_position = Point(x=1.0, y=2.0, z=3.0)
   # Set the new orientation
   new_hand_pose_action.new_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
   behavior_command.hand_pose_adaptation = new_hand_pose_action

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



# behavior_coordination.py

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


        # # Get available behaviors once at initialization
        # behavior_queue = msg.available_behaviors
        # print("Available behaviors:", behavior_queue)

        # if not behavior_queue:
        #     print("[ERROR] No available behaviors.")
        #     return

        # initialized = True
        # current_behavior_index = 0  # Start from the first behavior

        # # Monitor behavior execution
        # completed_behavior = msg.completed_behavior
        # if completed_behavior and completed_behavior in behavior_queue:
        #     print(f"Completed Behavior: {completed_behavior}")
        #     waiting_for_command = True
        #     current_behavior_index += 1  # Move to the next behavior

        # if current_behavior_index < len(behavior_queue) and waiting_for_command:
        #     # Execute the next behavior in the list
        #     behavior_command = AI2RCommandMessage()
        #     behavior_command.behavior_to_execute = behavior_queue[current_behavior_index]
        #     print(f"Commanded Behavior: {behavior_command.behavior_to_execute}")
        #     ros2["behavior_publisher"].publish(behavior_command)
            
        #     waiting_for_command = False  # Wait until the behavior completes
        #     initialized = True


    # if not initialized:
    #     # --------- Scene -----------
    #     scene_objects = msg.objects
    #     print("Objects in the scene:")
    #     #print("scene_objects: ",scene_objects)
    #     if scene_objects:  # This checks if the list is not empty
    #        for obj in scene_objects:
    #            id = obj.object_name
    #            print(f"{id}")
    #            pose_in_world = obj.object_pose_in_world
    #            pose_wrt_robot = obj.object_pose_in_robot_frame # This is the pose specified wrt to robot_pose
    #     else:
    #        print("-")

    #     # --------- Behaviors -----------
    #     behaviors = msg.available_behaviors
    #     print("Available behaviors:")
    #     if behaviors:
    #         for behavior in behaviors:
    #             print(behavior)
    #     else:
    #         print("-")

    # # --------- Reasoning -----------
    # # CAN DO SOME REASONING HERE based on objects in the scene and available behaviors

    # # --------- Monitoring -----------
    # completed_behavior = msg.completed_behavior
    # if not completed_behavior == "-":
    #    print("Completed Behavior: " + completed_behavior)
    #    waiting_for_command  = True
    # else:
    #    waiting_for_command  = False

    # failed_behavior = msg.failed_behavior
    # if failed_behavior:
    #    print("[FAILURE] Failed behavior: " + failure_behavior)
    #    # Failure details
    #    failure = msg.failure
    #    print("Description: " + failure.action_name)
    #    print("Type: " + failure.action_type)
    #    print("Frame: " + failure.action_frame)

    #    position_error = failure.position_error
    #    # Convert Point to numpy array
    #    error_vector = np.array([position_error.x, position_error.y, position_error.z])
    #    # Calculate the Euclidean norm (L2 norm)
    #    norm = np.linalg.norm(error_vector)
    #    print(f"The position error is: {norm}")
    #    orientation_error = failure.orientation_error

    #    position_tolerance = failure.position_tolerance
    #    orientation_tolerance = failure.orientation_tolerance

    # # --------- Reasoning -----------
    # # CAN DO SOME REASONING HERE based on failed behaviors

    # if waiting_for_command or not initialized:
    #    # --------- Coordination -----------
    #    behavior_command = AI2RCommandMessage()
    #    # DECIDE what behavior to execute based on reasoning. For example can decide to scan environment to detect objects
    #    behavior_command.behavior_to_execute = "GOTO"
    #    print("Commanded Behavior: " + behavior_command.behavior_to_execute)
    #    ros2["behavior_publisher"].publish(behavior_command)
    #    waiting_for_command = False
    #    initialized = True


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

# bevaior_coordination_test.py
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
waiting_for_command = True

def behavior_message_callback(msg):
    global initialized  # Access the global variables
    global waiting_for_command
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
    completed_behavior = msg.completed_behavior
    if not completed_behavior == "-":
       print("Completed Behavior: " + completed_behavior)
       waiting_for_command  = True
    else:
       waiting_for_command  = False

    failed_behavior = msg.failed_behavior
    if failed_behavior:
       print("[FAILURE] Failed behavior: " + failure_behavior)
       # Failure details
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

    # --------- Reasoning -----------
    # CAN DO SOME REASONING HERE based on failed behaviors

    if waiting_for_command or not initialized:
       # --------- Coordination -----------
       behavior_command = AI2RCommandMessage()
       # DECIDE what behavior to execute based on reasoning. For example can decide to scan environment to detect objects
       behavior_command.behavior_to_execute = "PLACE CHARGE ON DOOR"
       print("Commanded Behavior: " + behavior_command.behavior_to_execute)
       ros2["behavior_publisher"].publish(behavior_command)
       waiting_for_command = False
       initialized = True


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
