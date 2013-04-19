package chomp_motion_planner;

public interface GetChompCollisionCostRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "chomp_motion_planner/GetChompCollisionCostRequest";
  static final java.lang.String _DEFINITION = "# Array of string names for the links for which cost information is desired\nstring[] links\n\n# The state of the robot represented as an array of Kinematic states which\n# include the joint name and joint value\narm_navigation_msgs/RobotState state\n\n";
  java.util.List<java.lang.String> getLinks();
  void setLinks(java.util.List<java.lang.String> value);
  arm_navigation_msgs.RobotState getState();
  void setState(arm_navigation_msgs.RobotState value);
}
