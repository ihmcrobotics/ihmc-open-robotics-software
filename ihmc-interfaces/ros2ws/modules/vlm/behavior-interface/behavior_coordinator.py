"""
Behavior Coordinator for the explosive breaching demo.

Uses Qwen2.5-VL (served via vLLM) for all decision-making:
  - config.json               → plan the full behavior sequence
  - config_goto.json          → extract GOTO navigation parameters
  - config_scan.json          → extract SCAN target objects  (+ optional image)
  - config_receive_object.json→ extract RECEIVE OBJECT parameters

Usage:
    python behavior_coordinator.py
"""

import os
import re
import json
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
    AI2RScanMessage
)

from vlm_interface import VLMInterface

CONFIG_DIR = Path(__file__).parent


# ──────────────────────────────────────────────────────────────────────────────
# Scene utilities
# ──────────────────────────────────────────────────────────────────────────────

def parse_scene(msg):
    """Return (object_names, available_behaviors) from a status message."""
    names     = [obj.object_name for obj in msg.objects] if msg.objects else []
    behaviors = list(msg.available_behaviors) if msg.available_behaviors else []
    return names, behaviors


def log_failure(msg, log_file="failure_info.json"):
    """Extract failure info, append to JSON log, and return the info dict."""
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
# Response parsing
# ──────────────────────────────────────────────────────────────────────────────

def parse_json_response(response: str) -> dict:
    """
    Parse a JSON object from a VLM response that may be wrapped in markdown code fences.
    Handles both:
        ```json { ... } ```
        { ... }
    """
    # Strip markdown code fences if present
    stripped = re.sub(r"^```[a-z]*\n?", "", response.strip(), flags=re.IGNORECASE)
    stripped = re.sub(r"\n?```$", "", stripped.strip())
    # Normalize single quotes to double quotes for robustness
    stripped = stripped.replace("'", '"')
    return json.loads(stripped)


# ──────────────────────────────────────────────────────────────────────────────
# Plan parsing
# ──────────────────────────────────────────────────────────────────────────────

def behavior_list_to_planqueue(response: str) -> List[List[str]]:
    """
    Parse VLM output into [[behavior_name, full_step], ...].

    Expects the model to return something like:
        behavior_list = [
            SCAN,
            GOTO (to the person to the right of the barrier),
            RECEIVE OBJECT (the charge from the person),
            ...
        ]
    """
    match = re.search(r"behavior_list\s*=\s*\[(.*?)\]", response, re.DOTALL)
    if not match:
        print("Warning: no behavior_list found in VLM response.")
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
# Spatial object selection  (ported from behavior_nl_action.py)
# ──────────────────────────────────────────────────────────────────────────────

def _point_to_numpy(point: Point) -> np.ndarray:
    return np.array([point.x, point.y, point.z])


def _get_pose_by_name(
    name: str,
    scene_names: List[str],
    scene_poses: List[Point],
    robot_pose: Optional[Point] = None,
) -> Optional[Point]:
    if name.lower() == "robot" and robot_pose:
        return robot_pose
    if name in ("-", ""):
        return None
    try:
        return scene_poses[scene_names.index(name)]
    except ValueError:
        return None


def select_target_object(
    base_name: str,
    spatially_related_object: str,
    spatial_relation: str,
    class_discriminator: str,
    scene_object_names: List[str],
    scene_object_positions: List[Point],
    robot_pose: Optional[Point] = None,
) -> Optional[str]:
    """Resolve an ambiguous base name to a specific scene object using spatial geometry."""
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
# Behavior Coordinator
# ──────────────────────────────────────────────────────────────────────────────

class BehaviorCoordinator(Node):
    def __init__(self):
        super().__init__("behavior_coordination_node")

        # One VLMInterface per config
        self.vlm_planner = VLMInterface(CONFIG_DIR / "config.json")
        self.vlm_goto    = VLMInterface(CONFIG_DIR / "config_goto.json")
        self.vlm_scan    = VLMInterface(CONFIG_DIR / "config_scan.json")
        self.vlm_receive = VLMInterface(CONFIG_DIR / "config_receive_object.json")

        # State
        self.plan_queue       = []   # [[behavior_name, full_description], ...]
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

        self.get_logger().info("BehaviorCoordinator ready.")

    # ------------------------------------------------------------------
    # Status callback
    # ------------------------------------------------------------------

    def on_status(self, msg):
        scene_names, available_behaviors = parse_scene(msg)

        # Print scene on first message
        if not self.initialized:
            self.get_logger().info(f"Scene objects:       {scene_names}")
            self.get_logger().info(f"Available behaviors: {available_behaviors}")

        # Print each new completion
        if msg.completed_behavior != "-" and msg.completed_behavior != self.last_completion:
            self.get_logger().info(f"Completed: {msg.completed_behavior}")
            self.last_completion = msg.completed_behavior

        # Log failures once
        if msg.failed_behavior != "-" and not self.logged_failure:
            info = log_failure(msg)
            self.get_logger().warn(f"[FAILURE] {json.dumps(info, indent=2)}")
            self.logged_failure = True

        # Wait until robot is idle
        if msg.behavior_in_progress != "-":
            return

        # Wait for the last commanded behavior to be acknowledged as complete
        if self.next_behavior and msg.completed_behavior != self.next_behavior:
            return

        # Build the plan on the first idle tick
        if not self.plan_queue:
            if self.initialized:
                self.get_logger().info("Mission complete. All behaviors executed.")
                return
            self._plan_mission(msg)

        if not self.plan_queue:
            return

        # Send next command
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
    # Mission planning
    # ------------------------------------------------------------------

    def _plan_mission(self, msg):
        scene_names, available_behaviors = parse_scene(msg)

        vlm_input = (
            f"scene_objects: {scene_names}\n"
            f"available_behaviors: {available_behaviors}\n"
            f"previously_executed: {msg.completed_behavior if msg.completed_behavior != '-' else ''}\n"
            f"failed_behaviors: {msg.failed_behavior if msg.failed_behavior != '-' else ''}"
        )

        self.get_logger().info("Calling VLM mission planner...")
        self.vlm_planner.first_log_interaction(vlm_input)
        response = self.vlm_planner.call_model(vlm_input)
        self.get_logger().info(f"VLM plan:\n{response}")

        self.plan_queue = behavior_list_to_planqueue(response)
        if not self.plan_queue:
            self.get_logger().error("VLM returned no parseable behavior list.")

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
        # PLACE CHARGE ON DOOR and any other simple behaviors need no extra params
        return cmd

    def _build_scan(self, cmd, description, msg):
        # SCAN has no extra message parameters — the robot does a general scene scan.
        # We still call the VLM to log which objects it expects to find.
        cmd.adapting_behavior = True
        scene_names, _ = parse_scene(msg)
        vlm_input = (
            f"scene_objects: {scene_names}\n"
            f"task_description: {description}"
        )
        response = self.vlm_scan.call_model(vlm_input)
        self.get_logger().info(f"SCAN expected targets:\n{response}")

        try:
            data = parse_json_response(response)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Could not parse SCAN response ({e}).")
        scanMessage = AI2RScanMessage()
        data["target_objects"] = ["person",  "charge", "door_panel"]
        scanMessage.object_names =  data["target_objects"]
        cmd.scan = scanMessage

        return cmd

    def _build_goto(self, cmd, description, msg):
        cmd.adapting_behavior = True
        scene_names, _ = parse_scene(msg)
        scene_poses    = [obj.object_pose_in_world.position for obj in msg.objects]
        robot_pos      = msg.robot_mid_feet_under_pelvis_pose_in_world.position

        vlm_input = (
            f"scene_objects: {scene_names}\n"
            f"task_description: {description}"
        )
        response = self.vlm_goto.call_model(vlm_input)
        self.get_logger().info(f"GOTO params:\n{response}")

        try:
            data = parse_json_response(response)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Could not parse GOTO response ({e}).")
            return cmd

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
        scene_names, _ = parse_scene(msg)

        vlm_input = (
            f"scene_objects: {scene_names}\n"
            f"task_description: {description}"
        )
        response = self.vlm_receive.call_model(vlm_input)
        self.get_logger().info(f"RECEIVE OBJECT params:\n{response}")

        try:
            data = parse_json_response(response)
            receive_msg = AI2RReceiveObjectMessage()
            receive_msg.object_name = data["object_name"]
            receive_msg.side        = bytes([int(data["side"])])
            cmd.receive_object      = receive_msg
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Could not parse RECEIVE OBJECT response ({e}).")

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
