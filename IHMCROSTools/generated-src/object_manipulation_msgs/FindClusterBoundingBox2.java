package object_manipulation_msgs;

public interface FindClusterBoundingBox2 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/FindClusterBoundingBox2";
  static final java.lang.String _DEFINITION = "sensor_msgs/PointCloud2 cluster\n\n---\n\n#the pose of the box frame\ngeometry_msgs/PoseStamped pose\n\n#the dimensions of the box\ngeometry_msgs/Vector3 box_dims\n\n#whether there was an error\nint32 SUCCESS=0\nint32 TF_ERROR=1\nint32 error_code";
}
