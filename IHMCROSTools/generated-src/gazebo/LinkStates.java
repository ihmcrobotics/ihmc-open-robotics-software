package gazebo;

public interface LinkStates extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/LinkStates";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\n# broadcast all link states in world frame\nstring[] name                 # link names\ngeometry_msgs/Pose[] pose     # desired pose in world frame\ngeometry_msgs/Twist[] twist   # desired twist in world frame\n";
  java.util.List<java.lang.String> getName();
  void setName(java.util.List<java.lang.String> value);
  java.util.List<geometry_msgs.Pose> getPose();
  void setPose(java.util.List<geometry_msgs.Pose> value);
  java.util.List<geometry_msgs.Twist> getTwist();
  void setTwist(java.util.List<geometry_msgs.Twist> value);
}
