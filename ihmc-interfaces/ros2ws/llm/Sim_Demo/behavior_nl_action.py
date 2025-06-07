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
from behavior_msgs.msg import AI2RScanMessage
from behavior_msgs.msg import AI2RNavigationMessage
from behavior_msgs.msg import AI2RReceiveObjectMessage

import cv2
import numpy as np
from llm_interface import LLMInterface
import re
import sys
import json
from typing import List, Optional

ros2                = {}
initialized         = False
loggedFailure       = False
next_behavior       = ""
task_description    = ""
llm_call_counter    = 0
llm_plan            = []
plan_queue          = []
counter_retry = 0
retried_behavior = "-"

def behavior_message_callback(msg):
    global initialized  
    global loggedFailure
    global llm_call_counter
    global llm_plan
    global plan_queue
    global next_behavior
    global task_description
    global counter_retry
    global retried_behavior
    robot_pose = msg.robot_mid_feet_under_pelvis_pose_in_world

    if not initialized:
        # --------- Scene -----------
        scene_objects = msg.objects
        #print("Objects in the scene:")
        if scene_objects:  # This checks if the list is not empty
           for obj in scene_objects:
               id = obj.object_name
               #print(f"{id}")
               pose_in_world = obj.object_pose_in_world
               pose_wrt_robot = obj.object_pose_in_robot_frame # This is the pose specified wrt to robot_pose

        # --------- Behaviors -----------
        behaviors = msg.available_behaviors
        print("Available behaviors:")
        if behaviors:
            for behavior in behaviors:
                print(behavior)
        else:
            print("-")
        print("\n")
     # --------- Coordination -----------
    waiting_for_command = False

    # --------- Monitoring -----------
    failed_behavior = msg.failed_behavior
    failure = msg.failure
    if failed_behavior != "-" and loggedFailure == False:
        print("[FAILURE] -----------")
        print("Failed behavior: ")

        failure_info = {
            "Failed Behavior": failed_behavior,
            "Description": failure.action_name,
            "Type": failure.action_type,
            "Missing Frame" : "-",
            "Collision Object": "-",
            "Position Error": "false",
            "Retry #": 0,
            "Queue": "-"
        }

        if failure.missing_frame:
            failure_info["Missing Frame"] = failure.action_frame
        if failure.collision_name != "-":
            failure_info["Collision Object"] = failure.collision_name
        if counter_retry > 0:
            failure_info["Retry #"] = counter_retry

        position_error = failure.position_error
        error_vector = np.array([position_error.x, position_error.y, position_error.z])
        norm = np.linalg.norm(error_vector)
        if norm > failure.position_tolerance:
            failure_info["Position Error"] = "true"

        # Pretty-print the failure_info dictionary for the user
        print(json.dumps(failure_info, indent=4))


        # json_filename = 'failure_info.json'
        # with open(json_filename, 'a') as json_file:
        #     json.dump(failure_info, json_file, indent=4)
        json_filename = 'failure_info.json'

        # Load existing data if file exists
        if os.path.exists(json_filename):
            with open(json_filename, 'r') as json_file:
                try:
                    data = json.load(json_file)
                    if not isinstance(data, list):
                        data = []
                except json.JSONDecodeError:
                    data = []
        else:
            data = []

        # Run the check
        duplicate_user_input = find_duplicate_user_input(data, failure_info)
        is_input_duplicated = False

        if duplicate_user_input:
            print(f"Duplicate found! Existing queue: '{duplicate_user_input}'")
            queue = duplicate_user_input
            print("Queue after failure:", queue)

            # add this queue to infront of the existing plan_queue
            if queue:
                #print("Plan queue before failure:", plan_queue)
                plan_queue = queue + plan_queue
                print("Plan queue after failure:", plan_queue)
                if queue[0][0] == failed_behavior:
                    counter_retry = counter_retry + 1
                    retried_behavior = failed_behavior
                waiting_for_command     = True
                is_input_duplicated     = True
            else:
                print("No behaviors found in the LLM plan after failure. Exiting.")
                return
        else:
            print("No duplicate found. Safe to add the new entry.")
            # Ask the user how to proceed
            user_input = input("Type 'continue' to rerun or 'exit' to stop or 'next steps' to follow: ").strip().lower()

            #failure_info["User Input"] = user_input

        
        loggedFailure = True

        if not is_input_duplicated:
            if user_input == 'exit':
                print("Exiting due to failure.")
                exit(1)
            elif user_input == 'continue':
                print("rerunning the same command.")
                counter_retry = counter_retry + 1
                retried_behavior = next_behavior

                queue = [[next_behavior, task_description]]
                plan_queue = queue + plan_queue
                # Add next_behavior and task_description to front of plan_queue
                print("Plan queue after rerun:", plan_queue)
                waiting_for_command  = True
            else:
                task_description = user_input
                print("Next steps: ", task_description)
                print("Continuing after failure.")
                # Get all scene objects names
                scene_objects_names  = []
                scene_objects = msg.objects
                if scene_objects:  # This checks if the list is not empty
                    for obj in scene_objects:
                        scene_objects_names = [obj.object_name for obj in scene_objects]
                else:
                    print("No scene objects detected.")
                
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
                
                # Convert the input to a string for LLM processing
                llm_input       = str(llm_input)
                print("Loading config after failure")
                llm                 = LLMInterface(config_file="config.json")
                llm_input           = llm_input + "\ntask_description : " + task_description
                response            = llm.call_model(llm_input)
                print(" --------- Output after failure: --------- \n", response)
                queue               = behavior_list_to_planqueue(response)
                print("Queue after failure:", queue)

                # add this queue to infront of the existing plan_queue
                if queue:
                    #print("Plan queue before failure:", plan_queue)
                    plan_queue = queue + plan_queue
                    print("Plan queue after failure:", plan_queue)
                    waiting_for_command  = True
                else:
                    print("No behaviors found in the LLM plan after failure. Exiting.")
                    return
        print("Queue after failure:", queue)
        failure_info["Queue"] = queue        
            # Append new entry
        data.append(failure_info)
        # Write updated data
        with open(json_filename, 'w') as json_file:
            json.dump(data, json_file, indent=4)            

    if msg.behavior_in_progress == "-" and msg.completed_behavior == next_behavior:
       waiting_for_command  = True

    if waiting_for_command or not initialized:

        # Get all scene objects names
        scene_objects_names  = []
        scene_objects = msg.objects
        if scene_objects:  # This checks if the list is not empty
           for obj in scene_objects:
            #    id = obj.object_name
            #    pose_in_world = obj.object_pose_in_world
            #    pose_wrt_robot = obj.object_pose_in_robot_frame # This is the pose 
               scene_objects_names = [obj.object_name for obj in scene_objects]

        scene_objects_poses = [obj.object_pose_in_world for obj in msg.objects]
        scene_objects_positions = [pose.position for pose in scene_objects_poses]
        robot_position = msg.robot_mid_feet_under_pelvis_pose_in_world.position
        
        # Get all available behaviors
        available_behaviors = msg.available_behaviors
        completed_behavior  = msg.completed_behavior
        failed_behavior     = msg.failed_behavior
        if retried_behavior == completed_behavior:
            counter_retry = 0
            retried_behavior = "-"
        
        # Construct input for LLM decision-making
        llm_input = {
            # "task": current_task,
            "scene_objects": scene_objects_names,
            "available_behaviors": available_behaviors,
            "previously_executed": completed_behavior if completed_behavior != "-" else "",
            "failed_behaviors": failed_behavior if failed_behavior else "",
        }
        
        # Convert the input to a string for LLM processing
        llm_input       = str(llm_input)

        # Check if it is first time calling the LLM
        if llm_plan == []:
            llm             = LLMInterface(config_file="config.json")
            llm.first_log_interaction(llm_input)

            print(" --------- Calling the LLM for the first time --------- ")
            print("LLM Input for reasoning:", llm_input)
            print("\n")

            # Get the plan from the LLM
            response        = llm.call_model(llm_input)
            print(" --------- Output of LLM Planner: --------- \n", response)
            print("\n")
            llm_call_counter += 1

            plan_queue = behavior_list_to_planqueue(response)

            # Extract behavior list from the LLM plan
            match = re.search(r'behavior_list\s*=\s*\[(.*?)\]', response, re.DOTALL)
            if match:
                list_block = match.group(1)

                # Extract each behavior entry including content inside parentheses
                behaviors = re.findall(r'([^,]+(?:\([^\)]*\))?)', list_block)
                #print("Extracted Behaviors:", behaviors)

                # Step 3: Clean up whitespace
                llm_plan = [b.strip().rstrip(',') for b in behaviors]

                # Create list of [action, full_step]
                plan_queue = [[re.match(r'^([A-Z ]+)', step).group(1).strip(), step] for step in llm_plan]
            else:
                print("No behavior_list found.")

        if llm_plan == []:
            print("No behaviors found in the LLM plan. Exiting.")
            return
        else:
            # Get the next behavior from the plan queue
            if plan_queue:
                next_behavior, task_description = plan_queue.pop(0)
            else:
                print("Plan queue is empty. No next behavior to execute.")
                sys.exit(0)
            
        # After getting the plan from the LLM, we begin executing the next behavior
        behavior_command = AI2RCommandMessage()
        
        # DECIDE what behavior to execute based on reasoning. For example can decide to scan environment to detect objects
        behavior_command.behavior_to_execute = next_behavior
        behavior_command.adapting_behavior = False

#         if (behavior_command.behavior_to_execute == "SCAN"):
#             behavior_command.adapting_behavior = True
#             new_scan_behavior = AI2RScanMessage()
#             # Load the config file for SCAN action
#             print("Loading config for SCAN action")
#             llm                 = LLMInterface(config_file="config_scan.json")
#             llm_input           = ""
#             response            = llm.call_model(llm_input)
#             print(" --------- Output for SCAN action: --------- \n", response)
#             # Replace single quotes with double quotes and remove any trailing commas or leading/trailing whitespace
#             clean_response = response.replace("'", '"')
#
#             # Convert string to dictionary
#             data = json.loads(clean_response)
#
#             # Extract variables
#             target_objects = data['target_objects']
#             new_scan_behavior.target_objects = target_objects
#
#             behavior_command.scan = new_scan_behavior

        # Code for loading config files for different actions Eg: GOTO, RECEIVE, etc.
        if (behavior_command.behavior_to_execute == "GOTO"):

            behavior_command.adapting_behavior = True
            new_goto_behavior = AI2RNavigationMessage()
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

            # Extract variables
            target_object = data['target_object']
            spatial_relation_goto = data['spatial_relation_goto']
            pov_object_goto = data['pov_object_goto']
            spatially_related_object = data['spatially_related_object']
            spatial_relation_obj = data['spatial_relation_obj']
            class_discriminator = data['class_discriminator']

            if target_object in scene_objects_names:
                selected_object = target_object
            else:
                # If the target object is not in the scene objects, we need to select it based on spatial relation               
                selected_object = select_target_object(
                base_name= data['target_object'],
                spatially_related_object= data['spatially_related_object'],
                spatial_relation=data['spatial_relation_obj'],
                class_discriminator=data['class_discriminator'],
                scene_object_names=scene_objects_names,
                scene_object_positions=scene_objects_positions,
                robot_pose=robot_position,
                spatial_context_object="")
            
            print("Selected action target: ", selected_object)

            # Set the reference frame name - can copy from scene_objects.obj_name
            new_goto_behavior.target_object = selected_object
            # Set the distance to the object
            new_goto_behavior.distance_to_object = 1.0
            new_goto_behavior.pov_object = ""
            if hasattr(AI2RNavigationMessage, data['spatial_relation_goto']):
                new_goto_behavior.spatial_relation = getattr(AI2RNavigationMessage, data['spatial_relation_goto'])
            else:
                raise ValueError(f"Unknown spatial relation: {data['spatial_relation_goto']}")

            # new_goto_behavior.spatial_relation = data['spatial_relation']
            if new_goto_behavior.spatial_relation == AI2RNavigationMessage.DEFAULT or new_goto_behavior.pov_object == "":
                new_goto_behavior.pov_object = "walkingFrame"

            behavior_command.navigation = new_goto_behavior


        
        print("Commanded Behavior: " + behavior_command.behavior_to_execute)
        print("\n")
        ros2["behavior_publisher"].publish(behavior_command)
        initialized = True
        loggedFailure = False
        time.sleep(1)

# Function to check for duplicates
def find_duplicate_user_input(existing_entries, new_entry):
    keys_to_ignore = {"Queue"}
    
    for entry in existing_entries:
        # Compare only relevant keys
        is_duplicate = True
        for key in new_entry:
            if key in keys_to_ignore:
                continue
            # Normalize "Position error" vs "Position Error" casing
            existing_key = key
            # If any key mismatches, it's not a duplicate
            if entry.get(existing_key) != new_entry.get(key):
                is_duplicate = False
                break
        
        if is_duplicate:
            return entry["Queue"]  # Duplicate found
    
    return None  # No duplicate found


def behavior_list_to_planqueue(response):
    # Extract behavior list from the LLM plan
    match = re.search(r'behavior_list\s*=\s*\[(.*?)\]', response, re.DOTALL)
    if match:
        list_block = match.group(1)

        # Extract each behavior entry including content inside parentheses
        behaviors = re.findall(r'([^,]+(?:\([^\)]*\))?)', list_block)
        #print("Extracted Behaviors:", behaviors)

        # Step 3: Clean up whitespace
        plan = [b.strip().rstrip(',') for b in behaviors]

        # Create list of [action, full_step]
        queue = [[re.match(r'^([A-Z ]+)', step).group(1).strip(), step] for step in plan]
        return queue
        #print("Plan Queue:", plan_queue)
    else:
        queue = []
        print("No behavior_list found.")
        return queue

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
    class_discriminator: str,
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
#     print("candidates : ", candidates)
    if not candidates:
        print(f"No {base_name} objects found")
        return None
    if len(candidates) == 1:
        return candidates[0][0]

    # Get reference positions
    if spatially_related_object == "-" or  spatially_related_object == "":
            spatially_related_object = "Robot"

    ref_pose = get_pose_by_name(spatially_related_object, scene_object_names, scene_object_positions, robot_pose)
    if not ref_pose:
        print(f"Spatially related object '{spatially_related_object}' not found")
        return None

    if spatial_context_object == "-" or  spatial_context_object == "":
        spatial_context_object = "Robot"

    ctx_pose = get_pose_by_name(spatial_context_object, scene_object_names, scene_object_positions, robot_pose) if spatial_context_object else None
    if not ctx_pose:
        print(f"POV object '{spatial_context_object}' not found")
        return None

    # Handle DEFAULT relation
    if spatial_relation == "DEFAULT" or spatial_relation == "-":
        print("Distance : ", np.linalg.norm(point_to_numpy(candidates[0][1]) - point_to_numpy(ref_pose)))
        print("Distance : ", np.linalg.norm(point_to_numpy(candidates[1][1]) - point_to_numpy(ref_pose)))
        if class_discriminator == "CLOSE":
            return min(candidates, key=lambda x: np.linalg.norm(point_to_numpy(x[1]) - point_to_numpy(ref_pose)))[0]
        else:
            return max(candidates, key=lambda x: np.linalg.norm(point_to_numpy(x[1]) - point_to_numpy(ref_pose)))[0]

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
            if np.dot(offset, dir_norm) < -0.1:
                qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "FRONT":
            if np.dot(offset, dir_norm) > 0.1:
                qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "RIGHT":
            if np.dot(offset, left_vec) > 0.5:
                qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "LEFT":
            if np.dot(offset, left_vec) < -0.5:
                qualified.append((name, np.linalg.norm(offset)))
#     print("qualified ", qualified)

    if class_discriminator == "CLOSE":
        return min(qualified, key=lambda x: x[1])[0] if qualified else None
    else:
        return max(qualified, key=lambda x: x[1])[0] if qualified else None

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
