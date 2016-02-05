package object_manipulation_msgs;

public interface PlacePlanningResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/PlacePlanningResponse";
  static final java.lang.String _DEFINITION = "\n# the list of planned grasps\ngeometry_msgs/PoseStamped[] place_locations\n\n# whether an error occurred\nGraspPlanningErrorCode error_code";
  java.util.List<geometry_msgs.PoseStamped> getPlaceLocations();
  void setPlaceLocations(java.util.List<geometry_msgs.PoseStamped> value);
  object_manipulation_msgs.GraspPlanningErrorCode getErrorCode();
  void setErrorCode(object_manipulation_msgs.GraspPlanningErrorCode value);
}
