package point_cloud_server;

public interface StoreCloudActionGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "point_cloud_server/StoreCloudActionGoal";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalID goal_id\nStoreCloudGoal goal\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalID getGoalId();
  void setGoalId(actionlib_msgs.GoalID value);
  point_cloud_server.StoreCloudGoal getGoal();
  void setGoal(point_cloud_server.StoreCloudGoal value);
}
