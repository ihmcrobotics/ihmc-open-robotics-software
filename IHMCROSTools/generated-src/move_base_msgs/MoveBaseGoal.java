package move_base_msgs;

public interface MoveBaseGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "move_base_msgs/MoveBaseGoal";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\ngeometry_msgs/PoseStamped target_pose\n";
  geometry_msgs.PoseStamped getTargetPose();
  void setTargetPose(geometry_msgs.PoseStamped value);
}
