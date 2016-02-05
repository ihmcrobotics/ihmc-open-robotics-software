package arm_navigation_msgs;

public interface SetPlanningSceneDiffResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/SetPlanningSceneDiffResponse";
  static final java.lang.String _DEFINITION = "# This includes the current planning scene and any changes applied from the \n# diff\nPlanningScene planning_scene\n\n\t\t\t\t\t\t   \n\t\t\t\t   ";
  arm_navigation_msgs.PlanningScene getPlanningScene();
  void setPlanningScene(arm_navigation_msgs.PlanningScene value);
}
