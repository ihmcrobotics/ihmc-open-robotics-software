package object_manipulation_msgs;

public interface GraspableObject extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspableObject";
  static final java.lang.String _DEFINITION = "# an object that the object_manipulator can work on\n\n# a graspable object can be represented in multiple ways. This message\n# can contain all of them. Which one is actually used is up to the receiver\n# of this message. When adding new representations, one must be careful that\n# they have reasonable lightweight defaults indicating that that particular\n# representation is not available.\n\n# the tf frame to be used as a reference frame when combining information from\n# the different representations below\nstring reference_frame_id\n\n# potential recognition results from a database of models\n# all poses are relative to the object reference pose\nhousehold_objects_database_msgs/DatabaseModelPose[] potential_models\n\n# the point cloud itself\nsensor_msgs/PointCloud cluster\n\n# a region of a PointCloud2 of interest\nobject_manipulation_msgs/SceneRegion region\n\n# the name that this object has in the collision environment\nstring collision_name";
  java.lang.String getReferenceFrameId();
  void setReferenceFrameId(java.lang.String value);
  java.util.List<household_objects_database_msgs.DatabaseModelPose> getPotentialModels();
  void setPotentialModels(java.util.List<household_objects_database_msgs.DatabaseModelPose> value);
  sensor_msgs.PointCloud getCluster();
  void setCluster(sensor_msgs.PointCloud value);
  object_manipulation_msgs.SceneRegion getRegion();
  void setRegion(object_manipulation_msgs.SceneRegion value);
  java.lang.String getCollisionName();
  void setCollisionName(java.lang.String value);
}
