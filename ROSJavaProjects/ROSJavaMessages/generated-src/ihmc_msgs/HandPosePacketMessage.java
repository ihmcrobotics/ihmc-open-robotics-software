package ihmc_msgs;

public interface HandPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandPosePacketMessage";
  static final java.lang.String _DEFINITION = "## HandPosePacketMessage\r\n# This message commands the controller to move an arm end effector to a given\r\n# position and orientation. On Atlas, the end effector position is considered\r\n# as the end of the hand attachment plate, which is about 10 cm from the end\r\n# of the wrist. The position/orientation may be specified in world frame, chest\r\n# frame, or desired joint angles.\r\n\r\n# Options for robotSide\r\nuint8 LEFT=0 # refers to the LEFT side of a robot\r\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\r\nuint8 robot_side\r\n\r\n# data_type specifies whether or not the IHMC Controller should use the pose fields\r\n# or the joint angles array for computing control output for the arms\r\n# Options for dataType\r\nuint8 HAND_POSE=0 # the hand pose will be represented by a hand position and a hand orientation (joint angles will be ignored)\r\nuint8 JOINT_ANGLES=1 # the joint angles contained in this package will be used for control (position and orientation will be ignored)\r\nuint8 data_type\r\n\r\n# when using HAND_POSE datatype commands, specify whether the pose should be held in world or chest frame. Note that regardless of the frame specified here the position and orientation must be expressed in world frame.\r\n# Options for referenceFrame\r\nuint8 CHEST=0 # frame attached to the chest of the robot\r\nuint8 WORLD=1 # world frame\r\nuint8 reference_frame\r\n\r\n# to_home_position can be used to move the arm end effectors back to their starting\r\n# position, defined as down and beside the robot with slightly bent elbows\r\nbool to_home_position\r\n\r\n# the position component of a HAND_POSE type message. See the data_type field.\r\ngeometry_msgs/Vector3 position\r\n\r\n# the orientation component of a HAND_POSE type message. See the data_type field.\r\ngeometry_msgs/Quaternion orientation\r\n\r\n# trajectory_time specifies how fast or how slow to move to the desired pose\r\nfloat64 trajectory_time\r\n\r\n# joint_angles specifies the desired arm joint angles in order for a JOINT_ANGLES type messages.For Atlas V5 the controller assumes joint angles will be given in the following order:\r\n# shoulder yaw, shoulder roll, elbow pitch, elbow roll, upper wrist pitch, wrist roll, lower wrist pitch\r\nfloat64[] joint_angles\r\n\r\n# Specifies whether or not the orientation of the hand should be controller during HAND_POSE commands.\r\nbool control_orientation\r\n\r\n\r\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte HAND_POSE = 0;
  static final byte JOINT_ANGLES = 1;
  static final byte CHEST = 0;
  static final byte WORLD = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  byte getDataType();
  void setDataType(byte value);
  byte getReferenceFrame();
  void setReferenceFrame(byte value);
  boolean getToHomePosition();
  void setToHomePosition(boolean value);
  geometry_msgs.Vector3 getPosition();
  void setPosition(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  double[] getJointAngles();
  void setJointAngles(double[] value);
  boolean getControlOrientation();
  void setControlOrientation(boolean value);
}
