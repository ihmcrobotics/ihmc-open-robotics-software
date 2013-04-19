package object_manipulation_msgs;

public interface FindClusterBoundingBox2Request extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/FindClusterBoundingBox2Request";
  static final java.lang.String _DEFINITION = "sensor_msgs/PointCloud2 cluster\n\n";
  sensor_msgs.PointCloud2 getCluster();
  void setCluster(sensor_msgs.PointCloud2 value);
}
