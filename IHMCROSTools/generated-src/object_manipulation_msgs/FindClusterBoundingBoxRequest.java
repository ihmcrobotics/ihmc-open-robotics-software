package object_manipulation_msgs;

public interface FindClusterBoundingBoxRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/FindClusterBoundingBoxRequest";
  static final java.lang.String _DEFINITION = "sensor_msgs/PointCloud cluster\n\n";
  sensor_msgs.PointCloud getCluster();
  void setCluster(sensor_msgs.PointCloud value);
}
