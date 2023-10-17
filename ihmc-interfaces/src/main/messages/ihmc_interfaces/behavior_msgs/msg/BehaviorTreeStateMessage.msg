# Gives the current state of the complete collection of behavior tree nodes.
# Publishing all behavior tree nodes in one message can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

byte ARM_JOINT_ANGLES_ACTION = 0
byte CHEST_ORIENTATION_ACTION =1
byte FOOTSTEP_PLAN_ACTION = 2
byte SAKE_HAND_COMMAND_ACTION = 3
byte HAND_POSE_ACTION = 4
byte HAND_WRENCH_ACTION = 5
byte PELVIS_HEIGHT_PITCH_ACTION = 6
byte WAIT_DURATION_ACTION = 7
byte WALK_ACTION = 8

# The ID to assign to the next instantiated node
uint32 next_id

# A depth first ordered list of types.
byte[<=1000] behavior_tree_types

# A depth first ordered list of node indexes.
# The index is of that node in it's respective list for
# it's type.
uint32[<=1000] behavior_tree_indices

behavior_msgs/ArmJointAnglesActionStateMessage[<=200] arm_joint_angles_actions

behavior_msgs/ChestOrientationActionStateMessage[<=200] chest_orientation_actions

behavior_msgs/FootstepPlanActionStateMessage[<=200] footstep_plan_actions

behavior_msgs/SakeHandCommandActionStateMessage[<=200] sake_hand_command_actions

behavior_msgs/HandPoseActionStateMessage[<=200] hand_pose_actions

behavior_msgs/HandWrenchActionStateMessage[<=200] hand_wrench_actions

behavior_msgs/PelvisHeightPitchActionStateMessage[<=200] pelvis_height_actions

behavior_msgs/WaitDurationActionStateMessage[<=200] wait_duration_actions

behavior_msgs/WalkActionStateMessage[<=200] walk_actions
