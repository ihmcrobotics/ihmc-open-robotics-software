package navfn;

public interface MakeNavPlanRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "navfn/MakeNavPlanRequest";
  static final java.lang.String _DEFINITION = "geometry_msgs/PoseStamped start\ngeometry_msgs/PoseStamped goal\n";
  geometry_msgs.PoseStamped getStart();
  void setStart(geometry_msgs.PoseStamped value);
  geometry_msgs.PoseStamped getGoal();
  void setGoal(geometry_msgs.PoseStamped value);
}
