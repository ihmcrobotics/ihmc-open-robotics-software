package gazebo_msgs;

public interface WorldState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/WorldState";
  static final java.lang.String _DEFINITION = "# This is a message that holds data necessary to reconstruct a snapshot of the world\n#\n# = Approach to Message Passing =\n# The state of the world is defined by either\n#   1. Inertial Model pose, twist\n#      * kinematic data - connectivity graph from Model to each Link\n#      * joint angles\n#      * joint velocities\n#      * Applied forces - Body wrench\n#        * relative transform from Body to each collision Geom\n# Or\n#   2. Inertial (absolute) Body pose, twist, wrench\n#      * relative transform from Body to each collision Geom - constant, so not sent over wire\n#      * back compute from canonical body info to get Model pose and twist.\n#\n# Chooing (2.) because it matches most physics engines out there\n#   and is simpler.\n#\n# = Future =\n# Consider impacts on using reduced coordinates / graph (parent/child links) approach\n#   constraint and physics solvers.\n#\n# = Application =\n# This message is used to do the following:\n#   * reconstruct the world and objects for sensor generation\n#   * stop / start simulation - need pose, twist, wrench of each body\n#   * collision detection - need pose of each collision geometry.  velocity/acceleration if \n#\n# = Assumptions =\n# Assuming that each (physics) processor node locally already has\n#   * collision information - Trimesh for Geoms, etc\n#   * relative transforms from Body to Geom - this is assumed to be fixed, do not send oved wire\n#   * inertial information - does not vary in time\n#   * visual information - does not vary in time\n#\n\nHeader header\n\nstring[] name\ngeometry_msgs/Pose[] pose\ngeometry_msgs/Twist[] twist\ngeometry_msgs/Wrench[] wrench\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getName();
  void setName(java.util.List<java.lang.String> value);
  java.util.List<geometry_msgs.Pose> getPose();
  void setPose(java.util.List<geometry_msgs.Pose> value);
  java.util.List<geometry_msgs.Twist> getTwist();
  void setTwist(java.util.List<geometry_msgs.Twist> value);
  java.util.List<geometry_msgs.Wrench> getWrench();
  void setWrench(java.util.List<geometry_msgs.Wrench> value);
}
