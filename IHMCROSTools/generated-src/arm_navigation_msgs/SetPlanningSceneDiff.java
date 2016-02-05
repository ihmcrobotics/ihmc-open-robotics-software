package arm_navigation_msgs;

public interface SetPlanningSceneDiff extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/SetPlanningSceneDiff";
  static final java.lang.String _DEFINITION = "# OPTIONAL: This contains any differences the user would like \n# to apply to the current planning scene.  This will be \n# passed to other nodes. \nPlanningScene planning_scene_diff\n\n# OPTIONAL: Diff uses ordered collision operations in addition to allowed_collision_matrix\narm_navigation_msgs/OrderedCollisionOperations operations\n---\n# This includes the current planning scene and any changes applied from the \n# diff\nPlanningScene planning_scene\n\n\t\t\t\t\t\t   \n\t\t\t\t   \n";
}
