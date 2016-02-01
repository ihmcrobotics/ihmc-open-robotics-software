package household_objects_database_msgs;

public interface GetModelListResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelListResponse";
  static final java.lang.String _DEFINITION = "\n# the outcome of the query\nDatabaseReturnCode return_code\n\n# the actual model ids\nint32[] model_ids";
  household_objects_database_msgs.DatabaseReturnCode getReturnCode();
  void setReturnCode(household_objects_database_msgs.DatabaseReturnCode value);
  int[] getModelIds();
  void setModelIds(int[] value);
}
