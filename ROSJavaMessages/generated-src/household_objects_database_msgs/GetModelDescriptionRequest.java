package household_objects_database_msgs;

public interface GetModelDescriptionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelDescriptionRequest";
  static final java.lang.String _DEFINITION = "# retreieves various metadata for an model id\n\n# the id of the model\nint32 model_id\n\n";
  int getModelId();
  void setModelId(int value);
}
