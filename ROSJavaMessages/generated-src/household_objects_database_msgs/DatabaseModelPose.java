package household_objects_database_msgs;

public interface DatabaseModelPose extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/DatabaseModelPose";
  static final java.lang.String _DEFINITION = "# Informs that a specific model from the Model Database has been \n# identified at a certain location\n\n# the database id of the model\nint32 model_id\n\n# the pose that it can be found in\ngeometry_msgs/PoseStamped pose\n\n# a measure of the confidence level in this detection result\nfloat32 confidence\n\n# the name of the object detector that generated this detection result\nstring detector_name\n";
  int getModelId();
  void setModelId(int value);
  geometry_msgs.PoseStamped getPose();
  void setPose(geometry_msgs.PoseStamped value);
  float getConfidence();
  void setConfidence(float value);
  java.lang.String getDetectorName();
  void setDetectorName(java.lang.String value);
}
