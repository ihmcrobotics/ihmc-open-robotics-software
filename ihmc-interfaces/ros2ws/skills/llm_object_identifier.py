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
llm = LLMInterface(config_file="config_obj_identifier.json")

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
            spatially_related_object = lines[1]
            spatial_relation = lines[2]
            spatial_context_object = lines[3]

            selected_object = select_target_object(
                base_name=lines[0],
                spatially_related_object=lines[1],
                spatial_relation=lines[2],
                scene_object_names=scene_objects_names,
                scene_object_positions=scene_objects_positions,
                robot_pose=robot_position,
                spatial_context_object=lines[3])

            print(selected_object)

            # Increment the LLM call counter
            llm_call_counter += 1

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
