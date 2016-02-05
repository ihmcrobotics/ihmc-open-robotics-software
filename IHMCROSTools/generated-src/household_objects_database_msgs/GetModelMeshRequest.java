package household_objects_database_msgs;

public interface GetModelMeshRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelMeshRequest";
  static final java.lang.String _DEFINITION = "# retrieves the mesh for a model id\n\n# the id of the model\nint32 model_id\n\n";
  int getModelId();
  void setModelId(int value);
}
