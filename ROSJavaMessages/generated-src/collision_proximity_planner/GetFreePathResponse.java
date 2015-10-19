package collision_proximity_planner;

public interface GetFreePathResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collision_proximity_planner/GetFreePathResponse";
  static final java.lang.String _DEFINITION = "arm_navigation_msgs/RobotTrajectory robot_trajectory";
  arm_navigation_msgs.RobotTrajectory getRobotTrajectory();
  void setRobotTrajectory(arm_navigation_msgs.RobotTrajectory value);
}
