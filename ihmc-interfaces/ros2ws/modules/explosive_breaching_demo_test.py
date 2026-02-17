import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from behavior_msgs.msg import AI2RCommandMessage
from behavior_msgs.msg import AI2RStatusMessage
from behavior_msgs.msg import AI2RNavigationMessage
from behavior_msgs.msg import AI2RReceiveObjectMessage

import numpy as np
import json


class ExplosiveBreachingDemoNode(Node):
    def __init__(self):
        super().__init__('behavior_coordination_node')

        self.behavior_counter = 0
        self.initialized = False
        self.logged_failure = False
        self.last_commanded_behavior = None
        self.last_printed_completion = None
        self.demo_complete = False

        self.behaviors_baseline = [
            "SCAN", "GOTO", "RECEIVE OBJECT", "GOTO", "PLACE CHARGE ON DOOR", "GOTO"
        ]

        goto_person_param = ("Person1", AI2RNavigationMessage.DEFAULT, "")
        goto_door_param = ("DoorPanel1", AI2RNavigationMessage.FRONT, "")
        goto_barrier_param = ("Barrier1", AI2RNavigationMessage.BEHIND, "")
        receive_charge_param = ("Charge1",)
        self.parameters = [
            None, goto_person_param, receive_charge_param,
            goto_door_param, None, goto_barrier_param
        ]

        qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.behavior_subscriber = self.create_subscription(
            AI2RStatusMessage,
            '/ihmc/behavior_tree/ai2r_status',
            self.behavior_message_callback,
            qos_profile_best_effort
        )

        self.behavior_publisher = self.create_publisher(
            AI2RCommandMessage,
            '/ihmc/behavior_tree/ai2r_command',
            qos_profile_reliable
        )

    def behavior_message_callback(self, msg):
        if self.demo_complete:
            return

        if not self.initialized:
            # --------- Scene -----------
            scene_objects = msg.objects
            print("Objects in the scene:")
            if scene_objects:
                for obj in scene_objects:
                    print(f"{obj.object_name}")
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
        if (msg.completed_behavior != "-"
                and msg.behavior_in_progress == "-"
                and msg.completed_behavior != self.last_printed_completion):
            print("Completed Behavior:", msg.completed_behavior)
            self.last_printed_completion = msg.completed_behavior

        failed_behavior = msg.failed_behavior
        failure = msg.failure
        if failed_behavior != "-" and not self.logged_failure:
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
            self.logged_failure = True

        # --------- Coordination -----------
        waiting_for_command = msg.behavior_in_progress == "-"

        # Verify the last commanded behavior completed before sending the next
        if self.last_commanded_behavior is not None and not waiting_for_command:
            return
        if self.last_commanded_behavior is not None and waiting_for_command:
            if msg.completed_behavior != self.last_commanded_behavior:
                return

        if self.behavior_counter >= len(self.behaviors_baseline):
            if not self.demo_complete:
                print("Demo complete. All behaviors executed successfully.")
                self.demo_complete = True
            return

        if waiting_for_command or not self.initialized:
            behavior_command = AI2RCommandMessage()
            behavior_command.behavior_to_execute = self.behaviors_baseline[self.behavior_counter]
            behavior_command.adapting_behavior = False

            if behavior_command.behavior_to_execute == "GOTO":
                behavior_command.adapting_behavior = True
                new_goto_behavior = AI2RNavigationMessage()
                goto_parameters = self.parameters[self.behavior_counter]

                new_goto_behavior.target_object = goto_parameters[0]
                new_goto_behavior.distance_to_object = 1.0
                new_goto_behavior.pov_object = goto_parameters[2]
                new_goto_behavior.spatial_relation = goto_parameters[1]
                if new_goto_behavior.spatial_relation == AI2RNavigationMessage.DEFAULT or goto_parameters[2] == "":
                    new_goto_behavior.pov_object = "walkingFrame"

                behavior_command.navigation = new_goto_behavior

            if behavior_command.behavior_to_execute == "RECEIVE OBJECT":
                behavior_command.adapting_behavior = True
                new_receive_behavior = AI2RReceiveObjectMessage()
                new_receive_behavior.object_name = self.parameters[self.behavior_counter][0]
                new_receive_behavior.side = bytes([1])

                behavior_command.receive_object = new_receive_behavior

            print("Commanded Behavior: " + behavior_command.behavior_to_execute)
            self.behavior_publisher.publish(behavior_command)
            self.last_commanded_behavior = behavior_command.behavior_to_execute
            self.initialized = True
            self.logged_failure = False
            self.behavior_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ExplosiveBreachingDemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
