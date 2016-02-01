package object_manipulation_msgs;

public interface ClusterBoundingBox extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/ClusterBoundingBox";
  static final java.lang.String _DEFINITION = "# contains a bounding box, which is essentially a box somewhere in space\n# used here ususally for the outlier-invariant bounding box of a cluster of points\n\n#the pose of the box frame\ngeometry_msgs/PoseStamped pose_stamped\n\n#the dimensions of the box\ngeometry_msgs/Vector3 dimensions\n";
  geometry_msgs.PoseStamped getPoseStamped();
  void setPoseStamped(geometry_msgs.PoseStamped value);
  geometry_msgs.Vector3 getDimensions();
  void setDimensions(geometry_msgs.Vector3 value);
}
