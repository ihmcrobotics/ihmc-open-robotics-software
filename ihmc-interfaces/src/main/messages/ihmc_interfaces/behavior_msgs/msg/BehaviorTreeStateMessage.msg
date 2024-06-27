# Gives the current state of the complete collection of behavior tree nodes.
# Publishing all behavior tree nodes in one message can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

byte ROOT_NODE = 0
byte BASIC_NODE = 1
byte ACTION_SEQUENCE = 2
byte DOOR_TRAVERSAL = 3
byte TRASH_CAN_INTERACTION = 4
byte BUILDING_EXPLORATION = 5

byte CHEST_ORIENTATION_ACTION = 10
byte FOOTSTEP_PLAN_ACTION = 11
byte SAKE_HAND_COMMAND_ACTION = 12
byte HAND_POSE_ACTION = 13
byte HAND_WRENCH_ACTION = 14
byte SCREW_PRIMITIVE_ACTION = 15
byte PELVIS_HEIGHT_ORIENTATION_ACTION = 16
byte WAIT_DURATION_ACTION = 17
byte FOOT_POSE_ACTION = 18

# Monotonically increasing message ID that matches the CRDTInfo update number
uint32 sequence_id

# The ID to assign to the next instantiated node
uint32 next_id

# A mechanism for confirming and ending a freeze early
ihmc_common_msgs/ConfirmableRequestMessage confirmable_request

# A depth first ordered list of types.
byte[<=1000] behavior_tree_types

# A depth first ordered list of node indexes.
# The index is of that node in it's respective list for
# it's type.
uint32[<=1000] behavior_tree_indices

behavior_msgs/BehaviorTreeRootNodeStateMessage[<=1] root_nodes

behavior_msgs/BasicNodeStateMessage[<=200] basic_nodes

behavior_msgs/ActionSequenceStateMessage[<=200] action_sequences

behavior_msgs/DoorTraversalStateMessage[<=200] door_traversals

behavior_msgs/TrashCanInteractionStateMessage[<=200] trash_can_interactions

behavior_msgs/BuildingExplorationStateMessage[<=200] building_explorations

behavior_msgs/ChestOrientationActionStateMessage[<=200] chest_orientation_actions

behavior_msgs/FootstepPlanActionStateMessage[<=200] footstep_plan_actions

behavior_msgs/SakeHandCommandActionStateMessage[<=200] sake_hand_command_actions

behavior_msgs/HandPoseActionStateMessage[<=200] hand_pose_actions

behavior_msgs/HandWrenchActionStateMessage[<=200] hand_wrench_actions

behavior_msgs/ScrewPrimitiveActionStateMessage[<=200] screw_primitive_actions

behavior_msgs/PelvisHeightOrientationActionStateMessage[<=200] pelvis_height_actions

behavior_msgs/WaitDurationActionStateMessage[<=200] wait_duration_actions

behavior_msgs/FootPoseActionStateMessage[<=200] foot_pose_actions
