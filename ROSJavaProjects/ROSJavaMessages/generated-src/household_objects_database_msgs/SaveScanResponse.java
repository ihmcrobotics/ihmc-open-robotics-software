package household_objects_database_msgs;

public interface SaveScanResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/SaveScanResponse";
  static final java.lang.String _DEFINITION = "\n# the outcome of the query\nDatabaseReturnCode return_code";
  household_objects_database_msgs.DatabaseReturnCode getReturnCode();
  void setReturnCode(household_objects_database_msgs.DatabaseReturnCode value);
}
