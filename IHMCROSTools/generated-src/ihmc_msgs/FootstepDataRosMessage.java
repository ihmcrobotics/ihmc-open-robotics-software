package ihmc_msgs;

public interface FootstepDataRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataRosMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataRosMessage\n# This message specifies the position, orientation and side (left or right) of a desired footstep in\n# world frame.\n\n# Specifies which foot will swing to reach the foostep.\nuint8 robot_side\n\n# Specifies the position of the footstep (sole frame) in world frame.\ngeometry_msgs/Point location\n\n# Specifies the orientation of the footstep (sole frame) in world frame.\ngeometry_msgs/Quaternion orientation\n\n# predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n# the world. A value of null or an empty list will default to using the entire foot. Contact points\n# are expressed in sole frame. This ordering does not matter. For example: to tell the controller to\n# use the entire foot, the predicted contact points would be: predicted_contact_points: - {x: 0.5 *\n# foot_length, y: -0.5 * toe_width} - {x: 0.5 * foot_length, y: 0.5 * toe_width} - {x: -0.5 *\n# foot_length, y: -0.5 * heel_width} - {x: -0.5 * foot_length, y: 0.5 * heel_width} \nihmc_msgs/Point2dRosMessage[] predicted_contact_points\n\n# This contains information on what the swing trajectory should be for each step. Recomended is\n# DEFAULT.\nuint8 trajectory_type\n\n# Contains information on how high the robot should swing its foot. This affects trajectory types\n# DEFAULT and OBSTACLE_CLEARANCE.If a value smaller then the minumal swing height is chosen (e.g. 0.0)\n# the swing height will be changed to a default value.\nfloat64 swing_height\n\n# In case the trajectory type is set to CUSTOM two swing waypoints can be specified here. The\n# waypoints define sole positions.The controller will compute times and velocities at the waypoints.\n# This is a convinient way to shape the trajectory of the swing. If full control over the\n# swingtrajectory is desired use the trajectory type WAYPOINTS instead. The position waypoints are\n# expected in the trajectory frame.\ngeometry_msgs/Point[] position_waypoints\n\n# In case the trajectory type is set to WAYPOINTS, swing waypoints can be specified here. The\n# waypoints do not include thestart point (which is set to the current foot state at lift-off) and the\n# touch down point (which is specified by the location and orientation fields).All waypoints are for\n# the sole frame and expressed in the trajectory frame. The maximum number of points can be found in\n# the Footstep class.\nihmc_msgs/SE3TrajectoryPointRosMessage[] swing_trajectory\n\n# In case the trajectory type is set to WAYPOINTS, this value can be used to specify the trajectory\n# blend duration  in seconds. If greater than zero, waypoints that fall within the valid time window\n# (beginning at the start of the swing phase and spanning  the desired blend duration) will be\n# adjusted to account for the initial error between the actual and expected position and orientation\n# of the swing foot. Note that the expectedInitialLocation and expectedInitialOrientation fields must\n# be defined in order to enable trajectory blending.\nfloat64 swing_trajectory_blend_duration\n\n# The swingDuration is the time a foot is not in ground contact during a step. If the value of this\n# field is invalid (not positive) it will be replaced by a default swingDuration.\nfloat64 swing_duration\n\n# The transferDuration is the time spent with the feet in ground contact before a step. If the value\n# of this field is invalid (not positive) it will be replaced by a default transferDuration.\nfloat64 transfer_duration\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n# \"trajectory_type\" enum values:\nuint8 DEFAULT=0 # The controller will execute a default trajectory.\nuint8 OBSTACLE_CLEARANCE=1 # The controller will attempt to step on/off an obstacle.\nuint8 CUSTOM=2 # In this mode two trajectory position waypoints can be specified.\nuint8 WAYPOINTS=3 # The swing trajectory is fully defined by the given waypoints.\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte DEFAULT = 0;
  static final byte OBSTACLE_CLEARANCE = 1;
  static final byte CUSTOM = 2;
  static final byte WAYPOINTS = 3;
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Point getLocation();
  void setLocation(geometry_msgs.Point value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  java.util.List<ihmc_msgs.Point2dRosMessage> getPredictedContactPoints();
  void setPredictedContactPoints(java.util.List<ihmc_msgs.Point2dRosMessage> value);
  byte getTrajectoryType();
  void setTrajectoryType(byte value);
  double getSwingHeight();
  void setSwingHeight(double value);
  java.util.List<geometry_msgs.Point> getPositionWaypoints();
  void setPositionWaypoints(java.util.List<geometry_msgs.Point> value);
  java.util.List<ihmc_msgs.SE3TrajectoryPointRosMessage> getSwingTrajectory();
  void setSwingTrajectory(java.util.List<ihmc_msgs.SE3TrajectoryPointRosMessage> value);
  double getSwingTrajectoryBlendDuration();
  void setSwingTrajectoryBlendDuration(double value);
  double getSwingDuration();
  void setSwingDuration(double value);
  double getTransferDuration();
  void setTransferDuration(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
