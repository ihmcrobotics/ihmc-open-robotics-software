package household_objects_database_msgs;

public interface GetModelScans extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/GetModelScans";
  static final java.lang.String _DEFINITION = "# retrieves a list of object scans which match a given model id and source\n\n# the id of the model\nint32 model_id\n\n# the string name of the source of the scan data\nstring scan_source\n\n---\n\n# the outcome of the query\nDatabaseReturnCode return_code\n\n# the returned scans\nDatabaseScan[] matching_scans";
}
