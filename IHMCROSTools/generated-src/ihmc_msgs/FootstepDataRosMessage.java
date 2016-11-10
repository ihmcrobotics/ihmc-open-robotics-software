package ihmc_msgs;

public interface FootstepDataRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataRosMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataRosMessage\n# This message specifies the position, orientation and side (left or right) of a desired footstep in\n# world frame.\n\n# Specifies whether the given location is the location of the ankle or the sole.\nuint8 origin\n\n# Specifies which foot will swing to reach the foostep.\nuint8 robot_side\n\n# Specifies the position of the footstep. It is expressed in world frame.\ngeometry_msgs/Vector3 location\n\n# Specifies the orientation of the footstep. It is expressed in world frame.\ngeometry_msgs/Quaternion orientation\n\n# predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n# the world. A value of null or an empty list will default to using the entire foot. Contact points \n# are expressed in sole frame. This ordering does not matter. For example: to tell the controller to\n# use the entire foot, the predicted contact points would be: predicted_contact_points: - {x: 0.5 *\n# foot_length, y: -0.5 * toe_width} - {x: 0.5 * foot_length, y: 0.5 * toe_width} - {x: -0.5 *\n# foot_length, y: -0.5 * heel_width} - {x: -0.5 * foot_length, y: 0.5 * heel_width} \nihmc_msgs/Point2dRosMessage[] predicted_contact_points\n\n# This contains information on what the swing trajectory should be for each step. Recomended is\n# DEFAULT. \nuint8 trajectory_type\n\n# Contains information on how high the robot should step. This affects only basic and obstacle\n# clearance trajectories.Recommended values are between 0.1 (default) and 0.25. \nfloat64 swing_height\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"footstep_origin\" enum values:\nuint8 AT_ANKLE_FRAME=0 # The location of the footstep refers to the location of the ankle frame. The ankle frame is fixed in the foot, centered at the last ankle joint. The orientation = [qx = 0.0, qy = 0.0, qz = 0.0, qs = 1.0] corresponds to: x-axis pointing forward, y-axis pointing left, z-axis pointing upward. This option is for backward compatibility only and will be gone in an upcoming release. This origin is deprecated as it directly depends on the robot structure and is not directly related to the actual foot sole.\nuint8 AT_SOLE_FRAME=1 # The location of the footstep refers to the location of the sole frame. The sole frame is fixed in the foot, centered at the center of the sole. The orientation = [qx = 0.0, qy = 0.0, qz = 0.0, qs = 1.0] corresponds to: x-axis pointing forward, y-axis pointing left, z-axis pointing upward. This origin is preferred as it directly depends on the actual foot sole and is less dependent on the robot structure.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n# \"trajectory_type\" enum values:\nuint8 DEFAULT=0 # is a default trajectory\nuint8 BASIC=1 # will do a basic swing with the specified swing height\nuint8 PUSH_RECOVERY=2 # uses a low swing height for fast steps\nuint8 OBSTACLE_CLEARANCE=3 # will attempt to step over an obstacle\n\n";
  static final byte AT_ANKLE_FRAME = 0;
  static final byte AT_SOLE_FRAME = 1;
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte DEFAULT = 0;
  static final byte BASIC = 1;
  static final byte PUSH_RECOVERY = 2;
  static final byte OBSTACLE_CLEARANCE = 3;
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
  byte getTrajectoryType();
  void setTrajectoryType(byte value);
  double getSwingHeight();
  void setSwingHeight(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
