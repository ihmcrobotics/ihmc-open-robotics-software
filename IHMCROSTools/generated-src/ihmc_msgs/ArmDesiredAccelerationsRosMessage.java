package ihmc_msgs;

public interface ArmDesiredAccelerationsRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ArmDesiredAccelerationsRosMessage";
  static final java.lang.String _DEFINITION = "## ArmDesiredAccelerationsRosMessage\n# This message gives the user the option to bypass IHMC feedback controllers for the arm joints by\n# sending desired arm joint accelerations. One needs experience in control when activating the bypass\n# as it can result in unexpected behaviors for unreasonable accelerations. A message with a unique id\n# equals to 0 will be interpreted as invalid and will not be processed by the controller.\n\n# Specifies the side of the robot that will execute the trajectory.\nuint8 robot_side\n\n# Specifies the control mode for controlling the arm joints. See ArmControlMode.\nuint8 arm_control_mode\n\n# Specifies the desired joint accelerations. Only necessary when armControlMode == USER_CONTROL_MODE.\nfloat64[] arm_desired_joint_accelerations\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n# \"arm_control_mode\" enum values:\nuint8 IHMC_CONTROL_MODE=0 # The IHMC controller controls the arm joints to execute desired inputs given from ArmTrajectoryMessage, HandTrajectoryMessage, and WholeBodyTrajectoryMessage. PD controllers are run for the given inputs and will either compute the desired hand spatial acceleration or arm joint desired accelerations.The desired joint torques to achieve these desired accelerations are computed by the IHMC QP solver & inverse dynamics calculator.\nuint8 USER_CONTROL_MODE=1 # The user directly sets what the arm joint desired accelerations have to be. The IHMC controller will stop tracking positions and the user desired accelerations will be fed to the IHMC QP solver & inverse dynamics to compute the desired joint torques.\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte IHMC_CONTROL_MODE = 0;
  static final byte USER_CONTROL_MODE = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  byte getArmControlMode();
  void setArmControlMode(byte value);
  double[] getArmDesiredJointAccelerations();
  void setArmDesiredJointAccelerations(double[] value);
  long getUniqueId();
  void setUniqueId(long value);
}
