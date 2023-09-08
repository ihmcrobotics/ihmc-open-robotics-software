# Number of actions within the sequence
uint16 sequence_size

behavior_msgs/ArmJointAnglesActionMessage[<=200] arm_joint_angles_actions

behavior_msgs/BodyPartPoseActionMessage[<=200] chest_orientation_actions

behavior_msgs/FootstepPlanActionMessage[<=200] footstep_plan_actions

behavior_msgs/HandConfigurationActionMessage[<=200] hand_configuration_actions

behavior_msgs/SidedBodyPartPoseActionMessage[<=200] hand_pose_actions

behavior_msgs/HandWrenchActionMessage[<=200] hand_wrench_actions

behavior_msgs/BodyPartPoseActionMessage[<=200] pelvis_height_actions

behavior_msgs/WaitDurationActionMessage[<=200] wait_duration_actions

behavior_msgs/WalkActionMessage[<=200] walk_actions
