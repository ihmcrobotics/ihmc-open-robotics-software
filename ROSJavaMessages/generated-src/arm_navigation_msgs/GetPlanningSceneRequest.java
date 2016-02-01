package arm_navigation_msgs;

public interface GetPlanningSceneRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetPlanningSceneRequest";
  static final java.lang.String _DEFINITION = "# OPTIONAL: This contains any differences the user would like \n# to apply to the current planning scene.  This will not\n# affect the current state of any other node\nPlanningScene planning_scene_diff\n\n# OPTINAL: Diff uses ordered collision operations instead of allowed_collision_matrix\narm_navigation_msgs/OrderedCollisionOperations operations\n";
  arm_navigation_msgs.PlanningScene getPlanningSceneDiff();
  void setPlanningSceneDiff(arm_navigation_msgs.PlanningScene value);
  arm_navigation_msgs.OrderedCollisionOperations getOperations();
  void setOperations(arm_navigation_msgs.OrderedCollisionOperations value);
}
