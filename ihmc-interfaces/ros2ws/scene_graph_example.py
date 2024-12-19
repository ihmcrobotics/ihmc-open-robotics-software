import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from perception_msgs.msg import SceneGraphMessage
from perception_msgs.msg import SceneNodeMessage

import cv2
import numpy as np

ros2 = {}
state = {}

def scene_graph_message_callback(msg):
    print("Received SceneGraphMessage msg " + str(msg.sequence_id))
    
    if msg.sequence_id % 100 == 0:
        clear_scene_nodes()
        # add_simple_node()

    # Check if we are updating the scene graph
    if "updated_scene_graph" in state:
        ros2["scene_graph_publisher"].publish(state["updated_scene_graph"])
        state["scene_graph"] = state["updated_scene_graph"]
        del state["updated_scene_graph"]
    else:
        state["scene_graph"] = msg

def add_simple_node():
    print("Adding a new node to the scene graph")
    updated_scene_graph = state["scene_graph"]
    new_scene_node = SceneNodeMessage()
    new_scene_node.name = "Test new node"
    updated_scene_graph.scene_nodes.append(new_scene_node)
    state["updated_scene_graph"] = updated_scene_graph

def clear_scene_nodes():
    updated_scene_graph = state["scene_graph"]

    updated_scene_graph.scene_tree_types = [b'\x00']
    updated_scene_graph.scene_tree_indices = [0]

    root_node = updated_scene_graph.scene_nodes[0]
    root_node.number_of_children = 0
    updated_scene_graph.scene_nodes = [root_node]

    updated_scene_graph.detectable_scene_nodes = []
    updated_scene_graph.predefined_rigid_body_scene_nodes = []
    updated_scene_graph.aruco_marker_scene_nodes = []
    updated_scene_graph.centerpose_scene_nodes = []
    updated_scene_graph.static_relative_scene_nodes = []
    updated_scene_graph.primitive_rigid_body_scene_nodes = []
    updated_scene_graph.yolo_scene_nodes = []
    updated_scene_graph.door_scene_nodes = []
    updated_scene_graph.trash_can_nodes = []

    state["updated_scene_graph"] = updated_scene_graph

def main(args=None):
    rclpy.init(args=args)
    
    qos_profile_best_effort = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )

    node = rclpy.create_node('test_node')
    ros2["node"] = node

    scene_graph_subscriber = node.create_subscription(SceneGraphMessage,
                                            '/ihmc/scene_graph/status/scene_graph',
                                            scene_graph_message_callback, qos_profile_best_effort)
    ros2["scene_graph_subscriber"] = scene_graph_subscriber

    scene_graph_publisher = node.create_publisher(SceneGraphMessage, 
                                    '/ihmc/scene_graph/status/scene_graph',
                                    qos_profile_best_effort)
    ros2["scene_graph_publisher"] = scene_graph_publisher

    # Wait until interrupt
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == '__main__':
    main()
