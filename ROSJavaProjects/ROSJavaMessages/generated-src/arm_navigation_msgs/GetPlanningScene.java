package arm_navigation_msgs;

public interface GetPlanningScene extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetPlanningScene";
  static final java.lang.String _DEFINITION = "# OPTIONAL: This contains any differences the user would like \n# to apply to the current planning scene.  This will not\n# affect the current state of any other node\nPlanningScene planning_scene_diff\n\n# OPTINAL: Diff uses ordered collision operations instead of allowed_collision_matrix\narm_navigation_msgs/OrderedCollisionOperations operations\n---\n# This includes the current planning scene and any changes applied from the \n# diff\nPlanningScene planning_scene\n\n\t\t\t\t\t\t   \n\t\t\t\t   \n";
}
