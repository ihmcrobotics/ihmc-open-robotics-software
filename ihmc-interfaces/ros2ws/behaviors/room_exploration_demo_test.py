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
from behavior_msgs.msg import AI2RReceiveObjectMessage
from behavior_msgs.msg import AI2RPickUpObjectMessage

import cv2
import numpy as np
from typing import List
import json

ros2 = {}
initialized = False
loggedFailure = False

behavior_counter = 0
behaviors_baseline = ["SCAN", "GOTO", "PICK UP OBJECT"]

goto_table_param = ("Table1", AI2RNavigationMessage.DEFAULT, "", 0.8)
pickup_drill_param = ("Drill1",)
parameters = [None, goto_table_param, pickup_drill_param]

def behavior_message_callback(msg):
    global initialized  # Access the global variables
    global loggedFailure
    global behavior_counter
    global behaviors_baseline
    global parameters
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

    # --------- Monitoring -----------
    #if (msg.completed_behavior != "-"):
    print("Completed Behavior: " , msg.completed_behavior, " behavior_in_progress: " , msg.behavior_in_progress)
    failed_behavior = msg.failed_behavior
    failure = msg.failure
    if failed_behavior != "-" and loggedFailure == False:
        print("[FAILURE] -----------")
        print("Failed behavior: " + failed_behavior)

        failure_info = {
            "Failed behavior": failed_behavior,
            "Description": failure.action_name,
            "Type": failure.action_type
        }

        if failure.missing_frame:
            failure_info["Missing Frame"] = failure.reference_frame
        if failure.collision_name != "-":
            failure_info["Collision with"] = failure.collision_name
        position_error = failure.position_error
        error_vector = np.array([position_error.x, position_error.y, position_error.z])
        norm = np.linalg.norm(error_vector)
        if norm > failure.position_tolerance:
            failure_info["Position error"] = norm

        json_filename = 'failure_info.json'
        with open(json_filename, 'a') as json_file:
            json.dump(failure_info, json_file, indent=4)
        loggedFailure = True

    # --------- Coordination -----------
    waiting_for_command = False
    if msg.behavior_in_progress == "-" and msg.completed_behavior != "-":
       waiting_for_command  = True

    if waiting_for_command or not initialized:
        behavior_command = AI2RCommandMessage()
        # DECIDE what behavior to execute based on reasoning. For example can decide to scan environment to detect objects
        behavior_command.behavior_to_execute = behaviors_baseline[behavior_counter]
        behavior_command.adapting_behavior = False

        if (behavior_command.behavior_to_execute == "GOTO"):
            behavior_command.adapting_behavior = True
            new_goto_behavior = AI2RNavigationMessage()
            goto_parameters = parameters[behavior_counter]

            # Set the reference frame name - can copy from scene_objects.obj_name
            new_goto_behavior.target_object = goto_parameters[0]
            # Set the distance to the object
            new_goto_behavior.distance_to_object = goto_parameters[3]
            new_goto_behavior.pov_object = goto_parameters[2]
            new_goto_behavior.spatial_relation = goto_parameters[1]
            if new_goto_behavior.spatial_relation == AI2RNavigationMessage.DEFAULT or goto_parameters[2] == "":
                new_goto_behavior.pov_object = "walkingFrame"

            behavior_command.navigation = new_goto_behavior

        if (behavior_command.behavior_to_execute == "RECEIVE OBJECT"):
            behavior_command.adapting_behavior = True
            new_receive_behavior = AI2RReceiveObjectMessage()
            new_receive_behavior.object_name = parameters[behavior_counter][0]
            new_receive_behavior.side =  bytes([1])

            behavior_command.receive_object = new_receive_behavior

        if (behavior_command.behavior_to_execute == "PICK UP OBJECT"):
            behavior_command.adapting_behavior = True
            new_pickup_behavior = AI2RPickUpObjectMessage()
            new_pickup_behavior.object_name = parameters[behavior_counter][0]
            behavior_command.pickup_object = new_pickup_behavior

        print("Commanded Behavior: " + behavior_command.behavior_to_execute)
        ros2["behavior_publisher"].publish(behavior_command)
        initialized = True
        loggedFailure = False
        behavior_counter += 1
        print("Completed Behavior: " , msg.completed_behavior, " behavior_in_progress: " , msg.behavior_in_progress)

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
