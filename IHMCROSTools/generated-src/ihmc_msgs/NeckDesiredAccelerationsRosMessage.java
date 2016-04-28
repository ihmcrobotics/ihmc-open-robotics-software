package ihmc_msgs;

public interface NeckDesiredAccelerationsRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/NeckDesiredAccelerationsRosMessage";
  static final java.lang.String _DEFINITION = "## NeckDesiredAccelerationsRosMessage\n# This message gives the user the option to bypass IHMC feedback controllers for the neck joints by\n# sending desired neck joint accelerations. One needs experience in control when activating the bypass\n# as it can result in unexpected behaviors for unreasonable accelerations. A message with a unique id\n# equals to 0 will be interpreted as invalid and will not be processed by the controller.\n\n# Specifies the control mode for controlling the neck joints. See NeckControlMode.\nuint8 neck_control_mode\n\n# Specifies the desired joint accelerations. Only necessary when neckControlMode == USER_CONTROL_MODE.\nfloat64[] neck_desired_joint_accelerations\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"neck_control_mode\" enum values:\nuint8 IHMC_CONTROL_MODE=0 # The IHMC controller controls the arm joints to execute desired inputs given from NeckTrajectoryMessage, HeadTrajectoryMessage. PD controllers are run for the given inputs and will either compute the desired head spatial acceleration or neck joint desired accelerations.The desired joint torques to achieve these desired accelerations are computed by the IHMC QP solver & inverse dynamics calculator.\nuint8 USER_CONTROL_MODE=1 # The user directly sets what the neck joint desired accelerations have to be. The IHMC controller will stop tracking positions and the user desired accelerations will be fed to the IHMC QP solver & inverse dynamics to compute the desired joint torques.\n\n";
  static final byte IHMC_CONTROL_MODE = 0;
  static final byte USER_CONTROL_MODE = 1;
  byte getNeckControlMode();
  void setNeckControlMode(byte value);
  double[] getNeckDesiredJointAccelerations();
  void setNeckDesiredJointAccelerations(double[] value);
  long getUniqueId();
  void setUniqueId(long value);
}
