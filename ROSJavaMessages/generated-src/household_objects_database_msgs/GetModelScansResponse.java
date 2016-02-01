package household_objects_database_msgs;

public interface GetModelScansResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelScansResponse";
  static final java.lang.String _DEFINITION = "\n# the outcome of the query\nDatabaseReturnCode return_code\n\n# the returned scans\nDatabaseScan[] matching_scans";
  household_objects_database_msgs.DatabaseReturnCode getReturnCode();
  void setReturnCode(household_objects_database_msgs.DatabaseReturnCode value);
  java.util.List<household_objects_database_msgs.DatabaseScan> getMatchingScans();
  void setMatchingScans(java.util.List<household_objects_database_msgs.DatabaseScan> value);
}
