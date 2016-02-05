package collision_proximity_planner;

public interface GetFreePathRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collision_proximity_planner/GetFreePathRequest";
  static final java.lang.String _DEFINITION = "# A service definition for a standard forward kinematics service\narm_navigation_msgs/RobotState robot_state\n";
  arm_navigation_msgs.RobotState getRobotState();
  void setRobotState(arm_navigation_msgs.RobotState value);
}
