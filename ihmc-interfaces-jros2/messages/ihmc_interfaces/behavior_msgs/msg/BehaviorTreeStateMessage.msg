# Gives the current state of the complete collection of behavior tree nodes.
# Publishing all behavior tree nodes in one message can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

# Used to minimize bandwidth, nodes that are send
# without their full data.
byte PARTIAL_DATA = 0

byte ROOT_NODE = 1
byte BASIC_NODE = 2
byte ACTION_SEQUENCE = 3
byte FALLBACK_NODE = 4
byte CONDITION_NODE = 5
byte GOTO_NODE = 6
byte CHECKPOINT_NODE = 7
byte SCENE_ACTION = 8
byte AI2R_NODE = 9
byte DOOR_TRAVERSAL = 10
byte BUILDING_EXPLORATION = 11

byte NECK_ACTION = 12
byte SPINE_ACTION = 13
byte WALK_ACTION = 14
byte EZGRIPPER_ACTION = 15
byte ABILITY_HAND_ACTION = 16
byte ARM_ACTION = 17
byte HAND_WRENCH_ACTION = 18
byte PELVIS_ACTION = 20
byte WAIT_ACTION = 21
byte LEG_ACTION = 22
byte MIMIC_ACTION = 23

# Monotonically increasing message ID that matches the CRDTInfo update number
uint64 sequence_id

# The ID to assign to the next instantiated node
uint32 next_id

# The timestamp and modifier ID of the most recent time the root node was replaced or removed
ihmc_common_msgs/LatestModificationMessage latest_modification_to_root_reference

# The timestamp and modifier ID of the latest modification of the tree data fields
ihmc_common_msgs/LatestModificationMessage latest_modification_to_data

# A depth first ordered list of types.
byte[<=1000] behavior_tree_types

# A depth first ordered list of node indexes.
# The index is of that node in it's respective list for
# it's type.
uint32[<=1000] behavior_tree_indices

behavior_msgs/BasicNodeStateMessage[<=300] partial_data_nodes

behavior_msgs/BehaviorTreeRootNodeStateMessage[<=1] root_nodes

behavior_msgs/BasicNodeStateMessage[<=120] basic_nodes

behavior_msgs/ActionSequenceStateMessage[<=120] action_sequences

behavior_msgs/FallbackNodeStateMessage[<=120] fallback_nodes

behavior_msgs/ConditionNodeStateMessage[<=120] condition_nodes

behavior_msgs/GotoNodeStateMessage[<=120] goto_nodes

behavior_msgs/CheckpointNodeStateMessage[<=120] checkpoint_nodes

behavior_msgs/SceneActionStateMessage[<=120] scene_actions

behavior_msgs/AI2RNodeStateMessage[<=1] ai2r_nodes

behavior_msgs/DoorTraversalStateMessage[<=120] door_traversals

behavior_msgs/BuildingExplorationStateMessage[<=120] building_explorations

behavior_msgs/NeckActionStateMessage[<=120] neck_actions

behavior_msgs/SpineActionStateMessage[<=120] spine_actions

behavior_msgs/WalkActionStateMessage[<=120] walk_actions

behavior_msgs/EZGripperActionStateMessage[<=120] ez_gripper_actions

behavior_msgs/AbilityHandActionStateMessage[<=120] ability_hand_actions

behavior_msgs/ArmActionStateMessage[<=120] arm_actions

behavior_msgs/HandWrenchActionStateMessage[<=120] hand_wrench_actions

behavior_msgs/PelvisActionStateMessage[<=120] pelvis_actions

behavior_msgs/WaitActionStateMessage[<=120] wait_actions

behavior_msgs/LegActionStateMessage[<=120] leg_actions

behavior_msgs/MimicActionStateMessage[<=120] mimic_actions
