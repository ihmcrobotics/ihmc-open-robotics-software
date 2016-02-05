package household_objects_database_msgs;

public interface DatabaseScan extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/DatabaseScan";
  static final java.lang.String _DEFINITION = "# Contains the location of a stored point cloud scan of an object, \n# as well as additional metadata about that scan \n\n# the database id of the model\nint32 model_id\n\n# the location of the bag file storing the scan\nstring bagfile_location\n\n# the source of the scan (e.g. simulation)\nstring scan_source\n\n# the ground truth pose of the object that was scanned\ngeometry_msgs/PoseStamped pose\n\n# the topic that the points in the bag are published on\nstring cloud_topic";
  int getModelId();
  void setModelId(int value);
  java.lang.String getBagfileLocation();
  void setBagfileLocation(java.lang.String value);
  java.lang.String getScanSource();
  void setScanSource(java.lang.String value);
  geometry_msgs.PoseStamped getPose();
  void setPose(geometry_msgs.PoseStamped value);
  java.lang.String getCloudTopic();
  void setCloudTopic(java.lang.String value);
}
