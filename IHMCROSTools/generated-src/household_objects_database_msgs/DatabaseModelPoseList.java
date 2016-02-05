package household_objects_database_msgs;

public interface DatabaseModelPoseList extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/DatabaseModelPoseList";
  static final java.lang.String _DEFINITION = "# stores a list of possible database models recognition results\nDatabaseModelPose[] model_list";
  java.util.List<household_objects_database_msgs.DatabaseModelPose> getModelList();
  void setModelList(java.util.List<household_objects_database_msgs.DatabaseModelPose> value);
}
