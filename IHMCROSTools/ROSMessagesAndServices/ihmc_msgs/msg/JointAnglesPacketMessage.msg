## JointAnglesPacketMessage
# This message commands the IHMC joint position controller to move Atlas's joints to the desired angles.

# trajectoryTime specifies how fast or how slow to move to the desired joint angles
float64 trajectory_time

# neckJointAngle neck_ry
float64 neck_joint_angle

# spineJointAngles back_bky back_bkx back_bkz
float64[] spine_joint_angle

# rightLegJointAngle r_leg_hpz r_leg_hpx r_leg_hpy r_leg_kny r_leg_aky r_leg_akx
float64[] right_leg_joint_angle

# leftLegJointAngle l_leg_hpz l_leg_hpx l_leg_hpy l_leg_kny l_leg_aky l_leg_akx
float64[] left_leg_joint_angle

# rightArmJointAngle r_arm_shz r_arm_shx r_arm_ely r_arm_elx r_arm_wry r_arm_wrx r_arm_wry2
float64[] right_arm_joint_angle

# leftArmJointAngle l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2
float64[] left_arm_joint_angle

# spineJointLimits back_bky back_bkx back_bkz
int32[] spine_torque_limit

# rightLegJointTorqueLimit r_leg_hpz r_leg_hpx r_leg_hpy r_leg_kny r_leg_aky r_leg_akx
int32[] right_leg_torque_limit

# leftLegJointTorqueLimit l_leg_hpz l_leg_hpx l_leg_hpy l_leg_kny l_leg_aky l_leg_akx
int32[] left_leg_torque_limit

# rightArmTorqueLimit l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2
int32[] right_arm_torque_limit

# leftArmTorqueLimit l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2
int32[] left_arm_torque_limit

# keepLeftHandInTaskspacePosition specifies whether the position controller should try to maintain the left hand position in task space
bool keep_left_hand_in_taskspace_position

# keepRightHandInTaskspacePosition specifies whether the position controller should try to maintain the right hand position in task space
bool keep_right_hand_in_taskspace_position

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id


