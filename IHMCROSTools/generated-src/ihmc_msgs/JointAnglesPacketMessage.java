package ihmc_msgs;

public interface JointAnglesPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/JointAnglesPacketMessage";
  static final java.lang.String _DEFINITION = "## JointAnglesPacketMessage\n# This message commands the IHMC joint position controller to move Atlas\'s joints to the desired angles.\n\n# trajectoryTime specifies how fast or how slow to move to the desired joint angles\nfloat64 trajectory_time\n\n# neckJointAngle neck_ry\nfloat64 neck_joint_angle\n\n# spineJointAngles back_bky back_bkx back_bkz\nfloat64[] spine_joint_angle\n\n# rightLegJointAngle r_leg_hpz r_leg_hpx r_leg_hpy r_leg_kny r_leg_aky r_leg_akx\nfloat64[] right_leg_joint_angle\n\n# leftLegJointAngle l_leg_hpz l_leg_hpx l_leg_hpy l_leg_kny l_leg_aky l_leg_akx\nfloat64[] left_leg_joint_angle\n\n# rightArmJointAngle r_arm_shz r_arm_shx r_arm_ely r_arm_elx r_arm_wry r_arm_wrx r_arm_wry2\nfloat64[] right_arm_joint_angle\n\n# leftArmJointAngle l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2\nfloat64[] left_arm_joint_angle\n\n# spineJointLimits back_bky back_bkx back_bkz\nint32[] spine_torque_limit\n\n# rightLegJointTorqueLimit r_leg_hpz r_leg_hpx r_leg_hpy r_leg_kny r_leg_aky r_leg_akx\nint32[] right_leg_torque_limit\n\n# leftLegJointTorqueLimit l_leg_hpz l_leg_hpx l_leg_hpy l_leg_kny l_leg_aky l_leg_akx\nint32[] left_leg_torque_limit\n\n# rightArmTorqueLimit l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2\nint32[] right_arm_torque_limit\n\n# leftArmTorqueLimit l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2\nint32[] left_arm_torque_limit\n\n# keepLeftHandInTaskspacePosition specifies whether the position controller should try to maintain the left hand position in task space\nbool keep_left_hand_in_taskspace_position\n\n# keepRightHandInTaskspacePosition specifies whether the position controller should try to maintain the right hand position in task space\nbool keep_right_hand_in_taskspace_position\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  double getNeckJointAngle();
  void setNeckJointAngle(double value);
  double[] getSpineJointAngle();
  void setSpineJointAngle(double[] value);
  double[] getRightLegJointAngle();
  void setRightLegJointAngle(double[] value);
  double[] getLeftLegJointAngle();
  void setLeftLegJointAngle(double[] value);
  double[] getRightArmJointAngle();
  void setRightArmJointAngle(double[] value);
  double[] getLeftArmJointAngle();
  void setLeftArmJointAngle(double[] value);
  int[] getSpineTorqueLimit();
  void setSpineTorqueLimit(int[] value);
  int[] getRightLegTorqueLimit();
  void setRightLegTorqueLimit(int[] value);
  int[] getLeftLegTorqueLimit();
  void setLeftLegTorqueLimit(int[] value);
  int[] getRightArmTorqueLimit();
  void setRightArmTorqueLimit(int[] value);
  int[] getLeftArmTorqueLimit();
  void setLeftArmTorqueLimit(int[] value);
  boolean getKeepLeftHandInTaskspacePosition();
  void setKeepLeftHandInTaskspacePosition(boolean value);
  boolean getKeepRightHandInTaskspacePosition();
  void setKeepRightHandInTaskspacePosition(boolean value);
  long getUniqueId();
  void setUniqueId(long value);
}
