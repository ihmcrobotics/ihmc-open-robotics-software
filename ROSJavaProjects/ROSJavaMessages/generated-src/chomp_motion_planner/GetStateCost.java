package chomp_motion_planner;

public interface GetStateCost extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "chomp_motion_planner/GetStateCost";
  static final java.lang.String _DEFINITION = "# A service call to get the cost for a given state of the robot (specified in the form of a RobotState message)\n\n# A list of link names for which the cost should be computed\nstring[] link_names\n# The robot state for which the cost should be computed\narm_navigation_msgs/RobotState robot_state\n----\n# True if the cost computation was valid\nbool valid\n# The cost corresponding to each link\nfloat64[] costs\n";
}
