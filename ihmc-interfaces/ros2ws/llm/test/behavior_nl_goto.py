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
import json
from typing import List, Optional

# Calling the LLMInterface class for the first time
#llm                 = LLMInterface(config_file="config.json")

ros2                = {}
initialized         = False
waiting_for_command = True
llm_call_counter    = 0
llm_plan            = []
plan_queue          = []

def behavior_message_callback(msg):
    global initialized  # Access the global variables
    global waiting_for_command
    global llm_call_counter
    global llm_plan
    global plan_queue
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
        scene_objects_names  = []
        scene_objects = msg.objects
        if scene_objects:  # This checks if the list is not empty
           for obj in scene_objects:
               id = obj.object_name
               pose_in_world = obj.object_pose_in_world
               pose_wrt_robot = obj.object_pose_in_robot_frame # This is the pose specified wrt to robot_pose
               #print(f"{id} - Pose in World: {pose_in_world}, Pose wrt Robot: {pose_wrt_robot}")
               #insert id, pose_in_world, pose_wrt_robot into a dictionary or list if needed
            #    object_list = {
            #        "object_name": id
            #     #    "pose_in_world": {
            #     #        "pose" : pose_in_world.position,
            #     #        "orientation": pose_in_world.orientation
            #     #    },
            #     #    "pose_wrt_robot": {
            #     #         "pose": pose_wrt_robot.position,
            #     #         "orientation": pose_wrt_robot.orientation
            #     #    }
            #    }
            #    scene_objects_names.append(object_list)
               scene_objects_names = [obj.object_name for obj in scene_objects]

        scene_objects_poses = [obj.object_pose_in_world for obj in msg.objects]
        scene_objects_positions = [pose.position for pose in scene_objects_poses]
        robot_position = msg.robot_mid_feet_under_pelvis_pose_in_world.position

        # If calling llm for the first time, we can initialize the scene_objects_names as empty
        if llm_call_counter == 0:
            scene_objects_names = ''
        #print("Scene Objects:", scene_objects_names)
        
        # Get all available behaviors
        available_behaviors = msg.available_behaviors
        completed_behavior  = msg.completed_behavior
        failed_behavior     = msg.failed_behavior
        
        # Construct input for LLM decision-making
        llm_input = {
            # "task": current_task,
            "scene_objects": scene_objects_names,
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
            llm                 = LLMInterface(config_file="config.json")
            llm.first_log_interaction(llm_input)
            print(" --------- Calling the LLM for the first time --------- ")
            # Get the plan from the LLM
            response        = llm.call_model(llm_input)
            print(" --------- Output of LLM Planner: --------- \n", response)
            llm_call_counter += 1

            # Extract behavior list from the LLM plan
            match = re.search(r'behavior_list\s*=\s*\[(.*?)\]', response, re.DOTALL)
            if match:
                list_block = match.group(1)

                # Extract each behavior entry including content inside parentheses
                behaviors = re.findall(r'([^\n]+?\([^\)]+\))', list_block)

                # Step 3: Clean up whitespace
                llm_plan = [b.strip().rstrip(',') for b in behaviors]

                print('llm_plan : ', llm_plan)
                # Create list of [action, full_step]
                plan_queue = [[re.match(r'^(.+?)\s*\(', step).group(1).strip(), step] for step in llm_plan]
                print("Plan Queue:", plan_queue)
            else:
                print("No behavior_list found.")

        if llm_plan == []:
            print("No behaviors found in the LLM plan. Exiting.")
            return
        else:
            # Get the next behavior from the plan queue
            if plan_queue:
                next_behavior, task_description = plan_queue.pop(0)
                print("Next behavior from plan queue:", next_behavior)
                print("task_description:", task_description)
            else:
                print("Plan queue is empty. No next behavior to execute.")
                return

        # Code for loading config files for different actions
        if next_behavior == "GOTO":
            # Load the config file for GOTO action
            print("Loading config for GOTO action")
            llm                 = LLMInterface(config_file="config_goto.json")
            llm_input           = llm_input + "\ntask_description : " + task_description
            response            = llm.call_model(llm_input)
            print(" --------- Output for GOTO action: --------- \n", response)

            # print("type of response:", type(response))
            
            # Replace single quotes with double quotes and remove any trailing commas or leading/trailing whitespace
            clean_response = response.replace("'", '"')

            # Convert string to dictionary
            data = json.loads(clean_response)

            # # Extract variables
            # target_object = data['target_object']
            # spatially_related_object = data['spatially_related_object']
            # spatial_relation = data['spatial_relation']
            # pov_object = data['pov_object']

            # print(target_object)                # Person
            # print(spatially_related_object)     # Barrier1
            # print(spatial_relation)             # BEHIND
            # print(pov_object)                   # -

            selected_object = select_target_object(
                base_name= data['target_object'],
                spatially_related_object= data['spatially_related_object'],
                spatial_relation=data['spatial_relation'],
                scene_object_names=scene_objects_names,
                scene_object_positions=scene_objects_positions,
                robot_pose=robot_position,
                spatial_context_object=data['pov_object'])

            print("selected_object: ", selected_object)


        # # Query LLM for next action
        # print(" Calling the LLM ")
        # response        = llm.call_model(llm_input)

        # # Increment the LLM call counter
        # llm_call_counter += 1
        
        # print("LLM Response:", response, "type of response:", type(response), "length of response:", len(response))
        # print("LLM Call Counter:", llm_call_counter)

        # next_behavior   = response.strip('.\"')  # Remove any trailing punctuation or quotes
        # print("Next behavior:", next_behavior, "type of next behavior:", type(next_behavior), "length of next behavior:", len(next_behavior))

        # next_behavior   = '' # Remove any trailing punctuation or quotes
        # print("Next behavior:", next_behavior)

        # --------- Coordination -----------
        behavior_command = AI2RCommandMessage()
        # DECIDE what behavior to execute based on reasoning. For example can decide to scan environment to detect objects
        #behavior_command.behavior_to_execute = "GOTO CHARGE"
        behavior_command.behavior_to_execute = next_behavior
        print("Commanded Behavior: " + behavior_command.behavior_to_execute)
        ros2["behavior_publisher"].publish(behavior_command)
        waiting_for_command = False
        initialized = True  


def point_to_numpy(point: Point) -> np.ndarray:
    """Convert geometry_msgs/Point to numpy array"""
    return np.array([point.x, point.y, point.z])

def get_pose_by_name(
    object_name_to_find: str,
    scene_object_names: List[str],
    scene_object_poses: List[Point],
    robot_pose_world: Optional[Point] = None
) -> Optional[Point]:
    """Retrieve object position by name"""
    if object_name_to_find.lower() == "robot" and robot_pose_world:
        return robot_pose_world
    if object_name_to_find in ("-", ""):
        return None
    try:
        return scene_object_poses[scene_object_names.index(object_name_to_find)]
    except ValueError:
        print(f"Object '{object_name_to_find}' not found")
        return None

def select_target_object(
    base_name: str,
    spatially_related_object: str,
    spatial_relation: str,
    scene_object_names: List[str],
    scene_object_positions: List[Point],
    robot_pose: Optional[Point] = None,
    spatial_context_object: Optional[str] = None
) -> Optional[str]:
    """
    Selects target object based on spatial relationships
    Args:
        base_name: Object base name (e.g. "Cup")
        spatially_related_object: Reference object name (e.g. "Table1")
        spatial_relation: BEHIND/FRONT/LEFT/RIGHT/DEFAULT
        scene_object_names: List of all object names
        scene_object_positions: Corresponding object positions
        robot_pose: Robot's position (required if reference is robot)
        spatial_context_object: Secondary spatial reference
    Returns:
        Full object name (e.g. "Cup2") or None
    """
    # Find candidate objects
    candidates = [
        (name, pose) for name, pose in zip(scene_object_names, scene_object_positions)
        if name.startswith(base_name) and name[len(base_name):].isdigit()
    ]
    if not candidates:
        print(f"No {base_name} objects found")
        return None

    # Get reference positions
    ref_pose = get_pose_by_name(spatially_related_object, scene_object_names, scene_object_positions, robot_pose)
    if not ref_pose:
        print(f"Reference object '{spatially_related_object}' not found")
        return None

    if spatial_context_object == "-":
        spatial_context_object = "Robot"
    ctx_pose = get_pose_by_name(spatial_context_object, scene_object_names, scene_object_positions, robot_pose) if spatial_context_object else None

    # Handle DEFAULT relation
    if spatial_relation == "DEFAULT":
        return min(candidates, key=lambda x: np.linalg.norm(point_to_numpy(x[1]) - point_to_numpy(ref_pose)))[0]

    # Calculate spatial relationship
    ref_pos = point_to_numpy(ref_pose)
    ctx_pos = point_to_numpy(ctx_pose) if ctx_pose else ref_pos

    direction = ctx_pos - ref_pos
    if np.linalg.norm(direction) < 1e-6:
        print("Reference and context positions coincide")
        return candidates[0][0]

    dir_norm = direction / np.linalg.norm(direction)
    left_vec = np.cross(np.array([0, 0, 1]), dir_norm[:3])
    left_vec /= np.linalg.norm(left_vec)

    # Evaluate candidates
    qualified = []
    for name, pose in candidates:
        candidate_pos = point_to_numpy(pose)
        offset = candidate_pos - ref_pos

        if spatial_relation == "BEHIND":
            if np.dot(offset, dir_norm) < -0.7:
                qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "FRONT":
            if np.dot(offset, dir_norm) > 0.7:
                qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "RIGHT":
            if np.dot(offset, left_vec) > 0.7:
                qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "LEFT":
            if np.dot(offset, left_vec) < -0.7:
                qualified.append((name, np.linalg.norm(offset)))

    return min(qualified, key=lambda x: x[1])[0] if qualified else None



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
