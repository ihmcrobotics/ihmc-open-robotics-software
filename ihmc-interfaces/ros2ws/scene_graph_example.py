import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from perception_msgs.msg import SceneGraphMessage
from perception_msgs.msg import SceneNodeMessage
from perception_msgs.msg import PredefinedRigidBodySceneNodeMessage
from controller_msgs.msg import RigidBodyTransformMessage

import cv2
import numpy as np

ros2 = {}
state = {}
message_counter = 0

def scene_graph_message_callback(msg):
    print("Received SceneGraphMessage msg " + str(msg.sequence_id))
    global message_counter  # Access the global message counter

    message_counter += 1  # Increment the message counter
    state["scene_graph"] = msg
    # Occasionally add a node
    if message_counter % 100 == 0:
        print(msg)
        add_predefined_rigid_node()

    # Occasionally clear the scene nodes
    if msg.sequence_id % 500 == 0:
        clear_scene_nodes()

def add_simple_node():
    print("Adding a new node to the scene graph")
    updated_scene_graph = state["scene_graph"]

    new_scene_node = SceneNodeMessage()
    new_scene_node.id = updated_scene_graph.next_id
    updated_scene_graph.next_id = updated_scene_graph.next_id + 1 # We used next_id, so we have to increment it

    # Adding NewNode as a child of the root node
    root_node_index = 0
    root_node_number_of_children = updated_scene_graph.scene_nodes[root_node_index].number_of_children
    root_simple_node_number_of_children = root_node_number_of_children
    root_simple_node_number_of_children -= len(updated_scene_graph.detectable_scene_nodes)
    root_simple_node_number_of_children -= len(updated_scene_graph.predefined_rigid_body_scene_nodes)
    root_simple_node_number_of_children -= len(updated_scene_graph.aruco_marker_scene_nodes)
    root_simple_node_number_of_children -= len(updated_scene_graph.centerpose_scene_nodes)
    root_simple_node_number_of_children -= len(updated_scene_graph.static_relative_scene_nodes)
    root_simple_node_number_of_children -= len(updated_scene_graph.primitive_rigid_body_scene_nodes)
    root_simple_node_number_of_children -= len(updated_scene_graph.yolo_scene_nodes)
    root_simple_node_number_of_children -= len(updated_scene_graph.door_scene_nodes)
    root_simple_node_number_of_children -= len(updated_scene_graph.trash_can_nodes)

    new_scene_node.name = "NewNode" + str(root_node_number_of_children + 1)
    updated_scene_graph.scene_nodes[root_node_index].number_of_children = root_node_number_of_children + 1
    updated_scene_graph.scene_tree_types.insert(root_node_number_of_children + 1, b'\x00')
    updated_scene_graph.scene_tree_indices.insert(root_node_number_of_children + 1, root_simple_node_number_of_children + 1)
    updated_scene_graph.scene_nodes.append(new_scene_node)

    ros2["scene_graph_publisher"].publish(updated_scene_graph)
    print(updated_scene_graph)

def add_predefined_rigid_node():
    print("Adding a new node to the scene graph")
    updated_scene_graph = state["scene_graph"]

    new_scene_node = SceneNodeMessage()
    new_scene_node.id = updated_scene_graph.next_id
    updated_scene_graph.next_id = updated_scene_graph.next_id + 1 # We used next_id, so we have to increment it

    # Adding NewNode as a child of the root node
    root_node_index = 0
    root_node_number_of_children = updated_scene_graph.scene_nodes[root_node_index].number_of_children
    number_of_predefined_rigid_body_scene_nodes = len(updated_scene_graph.predefined_rigid_body_scene_nodes)

    new_scene_node.name = "PieceOfWood_" + str(number_of_predefined_rigid_body_scene_nodes + 1)

    new_predefined_rigid_body_scene_node = PredefinedRigidBodySceneNodeMessage()
    new_predefined_rigid_body_scene_node.scene_node = new_scene_node
    new_predefined_rigid_body_scene_node.initial_parent_id = root_node_index
    new_predefined_rigid_body_scene_node.initial_transform_to_parent = RigidBodyTransformMessage(x=0.0, y=0.0, z=0.0, m00=1.0, m01=0.0, m02=0.0, m10=0.0, m11=1.0, m12=0.0, m20=0.0, m21=0.0, m22=1.0)
    new_predefined_rigid_body_scene_node.visual_model_file_path = "environmentObjects/debris/2x4.g3dj"
    new_predefined_rigid_body_scene_node.visual_transform_to_parent = RigidBodyTransformMessage(x=0.0, y=0.0, z=0.0, m00=1.0, m01=0.0, m02=0.0, m10=0.0, m11=1.0, m12=0.0, m20=0.0, m21=0.0, m22=1.0)

    updated_scene_graph.scene_tree_types.insert(root_node_number_of_children + 1, b'\x02')
    updated_scene_graph.scene_tree_indices.insert(root_node_number_of_children + 1, number_of_predefined_rigid_body_scene_nodes)

    updated_scene_graph.scene_nodes[root_node_index].number_of_children = root_node_number_of_children + 1
    updated_scene_graph.predefined_rigid_body_scene_nodes.append(new_predefined_rigid_body_scene_node)

    ros2["scene_graph_publisher"].publish(updated_scene_graph)
    print(updated_scene_graph)

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

    ros2["scene_graph_publisher"].publish(updated_scene_graph)

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
