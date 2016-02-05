package object_manipulation_msgs;

public interface GraspPlanningResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspPlanningResponse";
  static final java.lang.String _DEFINITION = "\n# the list of planned grasps\nGrasp[] grasps\n\n# whether an error occurred\nGraspPlanningErrorCode error_code";
  java.util.List<object_manipulation_msgs.Grasp> getGrasps();
  void setGrasps(java.util.List<object_manipulation_msgs.Grasp> value);
  object_manipulation_msgs.GraspPlanningErrorCode getErrorCode();
  void setErrorCode(object_manipulation_msgs.GraspPlanningErrorCode value);
}
