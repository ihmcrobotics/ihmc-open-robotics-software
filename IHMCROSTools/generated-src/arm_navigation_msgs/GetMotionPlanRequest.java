package arm_navigation_msgs;

public interface GetMotionPlanRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetMotionPlanRequest";
  static final java.lang.String _DEFINITION = "# This service contains the definition for a request to the motion\n# planner and the output it provides\n\nMotionPlanRequest motion_plan_request\n\n";
  arm_navigation_msgs.MotionPlanRequest getMotionPlanRequest();
  void setMotionPlanRequest(arm_navigation_msgs.MotionPlanRequest value);
}
