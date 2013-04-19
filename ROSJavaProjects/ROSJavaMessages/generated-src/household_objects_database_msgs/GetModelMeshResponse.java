package household_objects_database_msgs;

public interface GetModelMeshResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelMeshResponse";
  static final java.lang.String _DEFINITION = "\n# the outcome of the query\nDatabaseReturnCode return_code\n\n# the returned mesh\nshape_msgs/Mesh mesh";
  household_objects_database_msgs.DatabaseReturnCode getReturnCode();
  void setReturnCode(household_objects_database_msgs.DatabaseReturnCode value);
  shape_msgs.Mesh getMesh();
  void setMesh(shape_msgs.Mesh value);
}
