package household_objects_database_msgs;

public interface GetModelListRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelListRequest";
  static final java.lang.String _DEFINITION = "# retrieves model id\'s from the database\n\n# optional: the name of a model set that the id\'s should belong to;\n# used to only retrieve a subset of models, pre-specified in the database\n# leave empty to get all available models\nstring model_set\n\n";
  java.lang.String getModelSet();
  void setModelSet(java.lang.String value);
}
