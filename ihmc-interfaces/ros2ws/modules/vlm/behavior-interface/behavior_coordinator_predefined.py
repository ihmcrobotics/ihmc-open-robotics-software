"""
Predefined Behavior Coordinator for the explosive breaching demo.

All LLM calls are replaced with hardcoded responses taken directly from
real VLM logs (Qwen2.5-VL-7B-Instruct-AWQ). No model server is required.

Usage:
    python behavior_coordinator_predefined.py
"""

import os
import re
import json
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point
from behavior_msgs.msg import (
    AI2RCommandMessage,
    AI2RStatusMessage,
    AI2RNavigationMessage,
    AI2RReceiveObjectMessage,
    AI2RScanMessage,
)


# ──────────────────────────────────────────────────────────────────────────────
# Hardcoded VLM responses (copied verbatim from vlm_logs.json)
# ──────────────────────────────────────────────────────────────────────────────

PREDEFINED_PLAN = (
    "behavior_list = ["
    "SCAN, "
    "GOTO (to the person to the right of the barrier), "
    "RECEIVE CHARGE, "
    "GOTO (to the front of the DoorPanel), "
    "PLACE CHARGE ON DOOR, "
    "GOTO (behind the barrier)"
    "]"
)

PREDEFINED_SCAN = {
    "target_objects": ["person", "charge", "traffic_barrier", "door_panel"]
}

# Keyed by a substring of the step description (lowercase).
# The coordinator matches the first key that appears in the description.
PREDEFINED_GOTO = {
    "person": {
        "target_object":          "person",
        "spatial_relation_goto":  "DEFAULT",
        "pov_object_goto":        "-",
        "spatially_related_object": "traffic_barrier",
        "spatial_relation_obj":   "RIGHT",
        "class_discriminator":    "CLOSE",
    },
    "doorpanel": {
        "target_object":          "door_panel",
        "spatial_relation_goto":  "FRONT",
        "pov_object_goto":        "-",
        "spatially_related_object": "-",
        "spatial_relation_obj":   "-",
        "class_discriminator":    "CLOSE",
    },
    "barrier": {
        "target_object":          "traffic_barrier",
        "spatial_relation_goto":  "BEHIND",
        "pov_object_goto":        "door_panel",
        "spatially_related_object": "-",
        "spatial_relation_obj":   "-",
        "class_discriminator":    "CLOSE",
    },
}

PREDEFINED_RECEIVE_CHARGE = {
    "object_name": "charge",
    "side": 1,
}


# ──────────────────────────────────────────────────────────────────────────────
# Scene utilities  (unchanged from behavior_coordinator.py)
# ──────────────────────────────────────────────────────────────────────────────

def parse_scene(msg):
    names     = [obj.object_name for obj in msg.objects] if msg.objects else []
    behaviors = list(msg.available_behaviors) if msg.available_behaviors else []
    return names, behaviors


def log_failure(msg, log_file="failure_info.json"):
    if msg.failed_behavior == "-":
        return None

    failure = msg.failure
    pos_err = failure.position_error
    norm    = float(np.linalg.norm([pos_err.x, pos_err.y, pos_err.z]))

    info = {
        "failed_behavior": msg.failed_behavior,
        "action_name":     failure.action_name,
        "action_type":     failure.action_type,
        "missing_frame":   failure.reference_frame if failure.missing_frame else None,
        "collision_with":  failure.collision_name  if failure.collision_name != "-" else None,
        "position_error":  norm if norm > failure.position_tolerance else None,
    }

    data = []
    if os.path.exists(log_file):
        with open(log_file) as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                data = []
    data.append(info)
    with open(log_file, "w") as f:
        json.dump(data, f, indent=4)

    return info


# ──────────────────────────────────────────────────────────────────────────────
# Plan parsing  (unchanged from behavior_coordinator.py)
# ──────────────────────────────────────────────────────────────────────────────

def behavior_list_to_planqueue(response: str) -> List[List[str]]:
    match = re.search(r"behavior_list\s*=\s*\[(.*?)\]", response, re.DOTALL)
    if not match:
        print("Warning: no behavior_list found.")
        return []

    list_block = match.group(1)
    items = re.findall(r"([^,]+(?:\([^\)]*\))?)", list_block)
    steps = [s.strip().rstrip(",") for s in items if s.strip()]

    queue = []
    for step in steps:
        m = re.match(r"^([A-Z][A-Z ]*?)(?:\s*\(|$)", step)
        if m:
            queue.append([m.group(1).strip(), step])

    return queue


# ──────────────────────────────────────────────────────────────────────────────
# Spatial object selection  (unchanged from behavior_coordinator.py)
# ──────────────────────────────────────────────────────────────────────────────

def _point_to_numpy(point: Point) -> np.ndarray:
    return np.array([point.x, point.y, point.z])


def _get_pose_by_name(name, scene_names, scene_poses, robot_pose=None):
    if name.lower() == "robot" and robot_pose:
        return robot_pose
    if name in ("-", ""):
        return None
    try:
        return scene_poses[scene_names.index(name)]
    except ValueError:
        return None


def select_target_object(
    base_name, spatially_related_object, spatial_relation, class_discriminator,
    scene_object_names, scene_object_positions, robot_pose=None,
) -> Optional[str]:
    candidates = [
        (name, pose)
        for name, pose in zip(scene_object_names, scene_object_positions)
        if name.startswith(base_name) and name[len(base_name):].isdigit()
    ]
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0][0]

    ref_name = spatially_related_object if spatially_related_object not in ("-", "") else "Robot"
    ref_pose = _get_pose_by_name(ref_name, scene_object_names, scene_object_positions, robot_pose)
    if not ref_pose:
        return None

    if spatial_relation in ("DEFAULT", "-", ""):
        key = lambda x: np.linalg.norm(_point_to_numpy(x[1]) - _point_to_numpy(ref_pose))
        return (min if class_discriminator == "CLOSE" else max)(candidates, key=key)[0]

    ref_pos   = _point_to_numpy(ref_pose)
    robot_pos = _point_to_numpy(robot_pose) if robot_pose else ref_pos
    direction = robot_pos - ref_pos
    if np.linalg.norm(direction) < 1e-6:
        return candidates[0][0]

    dir_norm = direction / np.linalg.norm(direction)
    left_vec = np.cross(np.array([0.0, 0.0, 1.0]), dir_norm[:3])
    if np.linalg.norm(left_vec) > 1e-6:
        left_vec /= np.linalg.norm(left_vec)

    qualified = []
    for name, pose in candidates:
        offset = _point_to_numpy(pose) - ref_pos
        if   spatial_relation == "BEHIND" and np.dot(offset, dir_norm) < -0.1:
            qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "FRONT"  and np.dot(offset, dir_norm) >  0.1:
            qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "RIGHT"  and np.dot(offset, left_vec) >  0.5:
            qualified.append((name, np.linalg.norm(offset)))
        elif spatial_relation == "LEFT"   and np.dot(offset, left_vec) < -0.5:
            qualified.append((name, np.linalg.norm(offset)))

    if not qualified:
        return None
    return (min if class_discriminator == "CLOSE" else max)(qualified, key=lambda x: x[1])[0]


# ──────────────────────────────────────────────────────────────────────────────
# Behavior Coordinator (predefined — no LLM calls)
# ──────────────────────────────────────────────────────────────────────────────

class BehaviorCoordinator(Node):
    def __init__(self):
        super().__init__("behavior_coordination_node")

        # State
        self.plan_queue       = []
        self.initialized      = False
        self.logged_failure   = False
        self.next_behavior    = ""
        self.next_description = ""
        self.last_completion  = None

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.status_sub = self.create_subscription(
            AI2RStatusMessage,
            "/ihmc/behavior_tree/ai2r_status",
            self.on_status,
            qos_be,
        )
        self.command_pub = self.create_publisher(
            AI2RCommandMessage,
            "/ihmc/behavior_tree/ai2r_command",
            qos_rel,
        )

        self.get_logger().info("BehaviorCoordinator (predefined) ready.")

    # ------------------------------------------------------------------
    # Status callback
    # ------------------------------------------------------------------

    def on_status(self, msg):
        scene_names, available_behaviors = parse_scene(msg)

        if not self.initialized:
            self.get_logger().info(f"Scene objects:       {scene_names}")
            self.get_logger().info(f"Available behaviors: {available_behaviors}")

        if msg.completed_behavior != "-" and msg.completed_behavior != self.last_completion:
            self.get_logger().info(f"Completed: {msg.completed_behavior}")
            self.last_completion = msg.completed_behavior

        if msg.failed_behavior != "-" and not self.logged_failure:
            info = log_failure(msg)
            self.get_logger().warn(f"[FAILURE] {json.dumps(info, indent=2)}")
            self.logged_failure = True

        if msg.behavior_in_progress != "-":
            return

        if self.next_behavior and msg.completed_behavior != self.next_behavior:
            return

        if not self.plan_queue:
            if self.initialized:
                self.get_logger().info("Mission complete. All behaviors executed.")
                return
            self._plan_mission()

        if not self.plan_queue:
            return

        self.next_behavior, self.next_description = self.plan_queue.pop(0)
        self.get_logger().info(
            f"Commanding [{self.next_behavior}]: {self.next_description}  "
            f"({len(self.plan_queue)} remaining)"
        )

        cmd = self._build_command(self.next_behavior, self.next_description, msg)
        self.command_pub.publish(cmd)
        self.initialized    = True
        self.logged_failure = False

    # ------------------------------------------------------------------
    # Mission planning — hardcoded
    # ------------------------------------------------------------------

    def _plan_mission(self):
        self.get_logger().info("[PREDEFINED] Using hardcoded mission plan.")
        self.get_logger().info(f"Plan: {PREDEFINED_PLAN}")
        self.plan_queue = behavior_list_to_planqueue(PREDEFINED_PLAN)
        if not self.plan_queue:
            self.get_logger().error("Failed to parse predefined plan.")

    # ------------------------------------------------------------------
    # Command builders
    # ------------------------------------------------------------------

    def _build_command(self, behavior: str, description: str, msg) -> AI2RCommandMessage:
        cmd = AI2RCommandMessage()
        cmd.behavior_to_execute = behavior
        cmd.adapting_behavior   = False

        if   behavior == "SCAN":           return self._build_scan(cmd, description, msg)
        elif behavior == "GOTO":           return self._build_goto(cmd, description, msg)
        elif behavior == "RECEIVE OBJECT": return self._build_receive_object(cmd, description, msg)
        return cmd

    def _build_scan(self, cmd, description, msg):
        cmd.adapting_behavior = True
        data = PREDEFINED_SCAN
        self.get_logger().info(f"[PREDEFINED] SCAN targets: {data['target_objects']}")

        scan_msg = AI2RScanMessage()
        scan_msg.object_names = data["target_objects"]
        cmd.scan = scan_msg
        return cmd

    def _build_goto(self, cmd, description, msg):
        cmd.adapting_behavior = True
        scene_names, _ = parse_scene(msg)
        scene_poses    = [obj.object_pose_in_world.position for obj in msg.objects]
        robot_pos      = msg.robot_mid_feet_under_pelvis_pose_in_world.position

        # Match predefined response by keyword in description
        desc_lower = description.lower()
        data = None
        for keyword, response in PREDEFINED_GOTO.items():
            if keyword in desc_lower:
                data = response
                break

        if data is None:
            self.get_logger().error(f"No predefined GOTO response for: '{description}'")
            return cmd

        self.get_logger().info(f"[PREDEFINED] GOTO params: {data}")

        target = data["target_object"]
        if target not in scene_names:
            target = select_target_object(
                base_name                = data["target_object"],
                spatially_related_object = data["spatially_related_object"],
                spatial_relation         = data["spatial_relation_obj"],
                class_discriminator      = data["class_discriminator"],
                scene_object_names       = scene_names,
                scene_object_positions   = scene_poses,
                robot_pose               = robot_pos,
            ) or data["target_object"]
        self.get_logger().info(f"GOTO resolved target: {target}")

        nav = AI2RNavigationMessage()
        nav.target_object      = target
        nav.distance_to_object = 1.0
        nav.pov_object         = ""

        relation_str = data["spatial_relation_goto"]
        if hasattr(AI2RNavigationMessage, relation_str):
            nav.spatial_relation = getattr(AI2RNavigationMessage, relation_str)
        else:
            self.get_logger().warn(f"Unknown spatial relation '{relation_str}', using DEFAULT.")
            nav.spatial_relation = AI2RNavigationMessage.DEFAULT

        if nav.spatial_relation == AI2RNavigationMessage.DEFAULT or not nav.pov_object:
            nav.pov_object = "walkingFrame"

        cmd.navigation = nav
        return cmd

    def _build_receive_object(self, cmd, description, msg):
        cmd.adapting_behavior = True
        data = PREDEFINED_RECEIVE_OBJECT
        self.get_logger().info(f"[PREDEFINED] RECEIVE OBJECT params: {data}")

        receive_msg = AI2RReceiveObjectMessage()
        receive_msg.object_name = data["object_name"]
        receive_msg.side        = bytes([int(data["side"])])
        cmd.receive_object      = receive_msg
        return cmd


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
