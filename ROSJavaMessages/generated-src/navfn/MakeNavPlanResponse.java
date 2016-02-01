package navfn;

public interface MakeNavPlanResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "navfn/MakeNavPlanResponse";
  static final java.lang.String _DEFINITION = "\nuint8 plan_found\nstring error_message\n\n# if plan_found is true, this is an array of waypoints from start to goal, where the first one equals start and the last one equals goal\ngeometry_msgs/PoseStamped[] path";
  byte getPlanFound();
  void setPlanFound(byte value);
  java.lang.String getErrorMessage();
  void setErrorMessage(java.lang.String value);
  java.util.List<geometry_msgs.PoseStamped> getPath();
  void setPath(java.util.List<geometry_msgs.PoseStamped> value);
}
