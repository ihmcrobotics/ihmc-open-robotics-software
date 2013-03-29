package nav_msgs;

public interface GetPlanRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nav_msgs/GetPlanRequest";
  static final java.lang.String _DEFINITION = "# Get a plan from the current position to the goal Pose \n\n# The start pose for the plan\ngeometry_msgs/PoseStamped start\n\n# The final pose of the goal position\ngeometry_msgs/PoseStamped goal\n\n# If the goal is obstructed, how many meters the planner can \n# relax the constraint in x and y before failing. \nfloat32 tolerance\n";
  geometry_msgs.PoseStamped getStart();
  void setStart(geometry_msgs.PoseStamped value);
  geometry_msgs.PoseStamped getGoal();
  void setGoal(geometry_msgs.PoseStamped value);
  float getTolerance();
  void setTolerance(float value);
}
