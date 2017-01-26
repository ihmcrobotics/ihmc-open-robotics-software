package ihmc_msgs;

public interface AdjustFootstepRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/AdjustFootstepRosMessage";
  static final java.lang.String _DEFINITION = "## AdjustFootstepRosMessage\n# The intent of this message is to adjust a footstep when the robot is executing it (a foot is\n# currently swinging to reach the footstep to be adjusted).\n\n# Specifies whether the given location is the location of the ankle or the sole.\nuint8 origin\n\n# Specifies which foot is expected to be executing the footstep to be adjusted.\nuint8 robot_side\n\n# Specifies the adjusted position of the footstep. It is expressed in world frame.\ngeometry_msgs/Vector3 location\n\n# Specifies the adjusted orientation of the footstep. It is expressed in world frame.\ngeometry_msgs/Quaternion orientation\n\n# predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n# the world. A value of null or an empty list will default to keep the contact points used for the\n# original footstep. Contact points  are expressed in sole frame. This ordering does not matter. For\n# example: to tell the controller to use the entire foot, the predicted contact points would be:\n# predicted_contact_points: - {x: 0.5 * foot_length, y: -0.5 * toe_width} - {x: 0.5 * foot_length, y:\n# 0.5 * toe_width} - {x: -0.5 * foot_length, y: -0.5 * heel_width} - {x: -0.5 * foot_length, y: 0.5 *\n# heel_width} \nihmc_msgs/Point2dRosMessage[] predicted_contact_points\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"footstep_origin\" enum values:\nuint8 AT_ANKLE_FRAME=0 # The location of the footstep refers to the location of the ankle frame. The ankle frame is fixed in the foot, centered at the last ankle joint. The orientation = [qx = 0.0, qy = 0.0, qz = 0.0, qs = 1.0] corresponds to: x-axis pointing forward, y-axis pointing left, z-axis pointing upward. This option is for backward compatibility only and will be gone in an upcoming release. This origin is deprecated as it directly depends on the robot structure and is not directly related to the actual foot sole.\nuint8 AT_SOLE_FRAME=1 # The location of the footstep refers to the location of the sole frame. The sole frame is fixed in the foot, centered at the center of the sole. The orientation = [qx = 0.0, qy = 0.0, qz = 0.0, qs = 1.0] corresponds to: x-axis pointing forward, y-axis pointing left, z-axis pointing upward. This origin is preferred as it directly depends on the actual foot sole and is less dependent on the robot structure.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n";
  static final byte AT_ANKLE_FRAME = 0;
  static final byte AT_SOLE_FRAME = 1;
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getOrigin();
  void setOrigin(byte value);
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getLocation();
  void setLocation(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  java.util.List<ihmc_msgs.Point2dRosMessage> getPredictedContactPoints();
  void setPredictedContactPoints(java.util.List<ihmc_msgs.Point2dRosMessage> value);
  long getUniqueId();
  void setUniqueId(long value);
}
