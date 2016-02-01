package household_objects_database_msgs;

public interface SaveScan extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/SaveScan";
  static final java.lang.String _DEFINITION = "# Stores information about a saved scan of an object to the database\n\n# the id of the model\nint32 scaled_model_id\n\n# The ground truth location of the object represented by the point cloud\ngeometry_msgs/PoseStamped ground_truth_pose\n\n# The location of the bagfile storing the scan\nstring bagfile_location\n\n# The string name of the source of the scan data\nstring scan_source\n\n# The topic in the bag that the cloud is published on \nstring cloud_topic\n\n---\n\n# the outcome of the query\nDatabaseReturnCode return_code";
}
