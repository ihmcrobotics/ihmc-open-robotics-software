package ihmc_msgs;

public interface HandPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandPosePacketMessage";
  static final java.lang.String _DEFINITION = "## HandPosePacketMessage\n# This message commands the controller to move an arm end effector to a given\n# position and orientation. On Atlas, the end effector position is considered\n# as the end of the hand attachment plate, which is about 10 cm from the end\n# of the wrist. The position/orientation may be specified in world frame, chest\n# frame, or desired joint angles.\n\n# Options for robotSide\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\nuint8 robot_side\n\n# data_type specifies whether or not the IHMC Controller should use the pose fields\n# or the joint angles array for computing control output for the arms\n# Options for dataType\nuint8 HAND_POSE=0 # the hand pose will be represented by a hand position and a hand orientation (joint angles will be ignored)\nuint8 JOINT_ANGLES=1 # the joint angles contained in this package will be used for control (position and orientation will be ignored)\nuint8 data_type\n\n# when using HAND_POSE datatype commands, specify whether the pose should be held in world or chest frame. Note that regardless of the frame specified here the position and orientation must be expressed in world frame.\n# Options for referenceFrame\nuint8 CHEST=0 # frame attached to the chest of the robot\nuint8 WORLD=1 # world frame\nuint8 reference_frame\n\n# to_home_position can be used to move the arm end effectors back to their starting\n# position, defined as down and beside the robot with slightly bent elbows\nbool to_home_position\n\n# the position component of a HAND_POSE type message. See the data_type field.\ngeometry_msgs/Vector3 position\n\n# the orientation component of a HAND_POSE type message. See the data_type field.\ngeometry_msgs/Quaternion orientation\n\n# trajectory_time specifies how fast or how slow to move to the desired pose\nfloat64 trajectory_time\n\n# joint_angles specifies the desired arm joint angles in order for a JOINT_ANGLES type messages.For Atlas V5 the controller assumes joint angles will be given in the following order:\n# shoulder yaw, shoulder roll, elbow pitch, elbow roll, upper wrist pitch, wrist roll, lower wrist pitch\nfloat64[] joint_angles\n\n# Specifies whether or not the orientation of the hand should be controller during HAND_POSE commands.\nbool control_orientation\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
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
  long getUniqueId();
  void setUniqueId(long value);
}
