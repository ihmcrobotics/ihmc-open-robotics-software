# Gives the current state of the complete collection of behavior tree nodes.
# Publishing all behavior tree nodes in one message can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

byte ROOT_NODE = 0
byte BASIC_NODE = 1
byte AI2R_NODE = 2
byte ACTION_SEQUENCE = 3
byte FALLBACK_NODE = 4
byte CONDITION_NODE = 5
byte GOTO_NODE = 6
byte DOOR_TRAVERSAL = 7
byte TRASH_CAN_INTERACTION = 8
byte BUILDING_EXPLORATION = 9

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
uint64 sequence_id

# The ID to assign to the next instantiated node
uint32 next_id

# A mechanism for confirming and ending a freeze early
ihmc_common_msgs/ConfirmableRequestMessage confirmable_request

# A depth first ordered list of types.
byte[] behavior_tree_types

# A depth first ordered list of node indexes.
# The index is of that node in it's respective list for
# it's type.
uint32[] behavior_tree_indices

behavior_msgs/BehaviorTreeRootNodeStateMessage root_node

behavior_msgs/BasicNodeStateMessage[] basic_nodes

behavior_msgs/AI2RNodeStateMessage[] ai2r_nodes

behavior_msgs/ActionSequenceStateMessage[] action_sequences

behavior_msgs/FallbackNodeStateMessage[] fallback_nodes

behavior_msgs/ConditionNodeStateMessage[] condition_nodes

behavior_msgs/GotoNodeStateMessage[] goto_nodes

behavior_msgs/DoorTraversalStateMessage[] door_traversals

behavior_msgs/TrashCanInteractionStateMessage[] trash_can_interactions

behavior_msgs/BuildingExplorationStateMessage[] building_explorations

behavior_msgs/ChestOrientationActionStateMessage[] chest_orientation_actions

behavior_msgs/FootstepPlanActionStateMessage[] footstep_plan_actions

behavior_msgs/SakeHandCommandActionStateMessage[] sake_hand_command_actions

behavior_msgs/HandPoseActionStateMessage[] hand_pose_actions

behavior_msgs/HandWrenchActionStateMessage[] hand_wrench_actions

behavior_msgs/ScrewPrimitiveActionStateMessage[] screw_primitive_actions

behavior_msgs/PelvisHeightOrientationActionStateMessage[] pelvis_height_actions

behavior_msgs/WaitDurationActionStateMessage[] wait_duration_actions

behavior_msgs/FootPoseActionStateMessage[] foot_pose_actions
