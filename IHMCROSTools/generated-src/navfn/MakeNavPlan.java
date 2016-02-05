package navfn;

public interface MakeNavPlan extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "navfn/MakeNavPlan";
  static final java.lang.String _DEFINITION = "geometry_msgs/PoseStamped start\ngeometry_msgs/PoseStamped goal\n---\n\nuint8 plan_found\nstring error_message\n\n# if plan_found is true, this is an array of waypoints from start to goal, where the first one equals start and the last one equals goal\ngeometry_msgs/PoseStamped[] path\n";
}
