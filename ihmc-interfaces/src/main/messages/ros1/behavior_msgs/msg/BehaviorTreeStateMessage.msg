# Gives the current state of the complete collection of behavior tree nodes.
# Publishing all behavior tree nodes in one message can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

int8 BASIC_NODE=0

int8 ACTION_SEQUENCE=1

int8 DOOR_TRAVERSAL=2

int8 BUILDING_EXPLORATION=3

int8 CHEST_ORIENTATION_ACTION=10

int8 FOOTSTEP_PLAN_ACTION=11

int8 SAKE_HAND_COMMAND_ACTION=12

int8 HAND_POSE_ACTION=13

int8 HAND_WRENCH_ACTION=14

int8 SCREW_PRIMITIVE_ACTION=15

int8 PELVIS_HEIGHT_PITCH_ACTION=16

int8 WAIT_DURATION_ACTION=17

# Monotonically increasing message ID that matches the CRDTInfo update number
uint32 sequence_id

# The ID to assign to the next instantiated node
uint32 next_id

# A mechanism for confirming and ending a freeze early
ihmc_common_msgs/ConfirmableRequestMessage confirmable_request

# A depth first ordered list of types.
int8[] behavior_tree_types

# A depth first ordered list of node indexes.
# The index is of that node in it's respective list for
# it's type.
uint32[] behavior_tree_indices

behavior_msgs/BasicNodeStateMessage[] basic_nodes

behavior_msgs/ActionSequenceStateMessage[] action_sequences

behavior_msgs/DoorTraversalStateMessage[] door_traversals

behavior_msgs/BuildingExplorationStateMessage[] building_explorations

behavior_msgs/ChestOrientationActionStateMessage[] chest_orientation_actions

behavior_msgs/FootstepPlanActionStateMessage[] footstep_plan_actions

behavior_msgs/SakeHandCommandActionStateMessage[] sake_hand_command_actions

behavior_msgs/HandPoseActionStateMessage[] hand_pose_actions

behavior_msgs/HandWrenchActionStateMessage[] hand_wrench_actions

behavior_msgs/ScrewPrimitiveActionStateMessage[] screw_primitive_actions

behavior_msgs/PelvisHeightPitchActionStateMessage[] pelvis_height_actions

behavior_msgs/WaitDurationActionStateMessage[] wait_duration_actions


