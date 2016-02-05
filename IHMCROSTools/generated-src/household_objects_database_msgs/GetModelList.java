package household_objects_database_msgs;

public interface GetModelList extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelList";
  static final java.lang.String _DEFINITION = "# retrieves model id\'s from the database\n\n# optional: the name of a model set that the id\'s should belong to;\n# used to only retrieve a subset of models, pre-specified in the database\n# leave empty to get all available models\nstring model_set\n\n---\n\n# the outcome of the query\nDatabaseReturnCode return_code\n\n# the actual model ids\nint32[] model_ids";
}
