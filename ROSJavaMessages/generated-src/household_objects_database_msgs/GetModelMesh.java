package household_objects_database_msgs;

public interface GetModelMesh extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelMesh";
  static final java.lang.String _DEFINITION = "# retrieves the mesh for a model id\n\n# the id of the model\nint32 model_id\n\n---\n\n# the outcome of the query\nDatabaseReturnCode return_code\n\n# the returned mesh\nshape_msgs/Mesh mesh\n";
}
