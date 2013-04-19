package household_objects_database_msgs;

public interface GetModelDescription extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelDescription";
  static final java.lang.String _DEFINITION = "# retreieves various metadata for an model id\n\n# the id of the model\nint32 model_id\n\n---\n\n# the outcome of the query\nDatabaseReturnCode return_code\n\n# the tags of the model\nstring[] tags\n\n# the name of the model\nstring name\n\n# the maker of the model\nstring maker\n\n";
}
