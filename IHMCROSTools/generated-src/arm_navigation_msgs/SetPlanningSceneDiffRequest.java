package arm_navigation_msgs;

public interface SetPlanningSceneDiffRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/SetPlanningSceneDiffRequest";
  static final java.lang.String _DEFINITION = "# OPTIONAL: This contains any differences the user would like \n# to apply to the current planning scene.  This will be \n# passed to other nodes. \nPlanningScene planning_scene_diff\n\n# OPTIONAL: Diff uses ordered collision operations in addition to allowed_collision_matrix\narm_navigation_msgs/OrderedCollisionOperations operations\n";
  arm_navigation_msgs.PlanningScene getPlanningSceneDiff();
  void setPlanningSceneDiff(arm_navigation_msgs.PlanningScene value);
  arm_navigation_msgs.OrderedCollisionOperations getOperations();
  void setOperations(arm_navigation_msgs.OrderedCollisionOperations value);
}
