package household_objects_database_msgs;

public interface GetModelDescriptionResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelDescriptionResponse";
  static final java.lang.String _DEFINITION = "\n# the outcome of the query\nDatabaseReturnCode return_code\n\n# the tags of the model\nstring[] tags\n\n# the name of the model\nstring name\n\n# the maker of the model\nstring maker";
  household_objects_database_msgs.DatabaseReturnCode getReturnCode();
  void setReturnCode(household_objects_database_msgs.DatabaseReturnCode value);
  java.util.List<java.lang.String> getTags();
  void setTags(java.util.List<java.lang.String> value);
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getMaker();
  void setMaker(java.lang.String value);
}
