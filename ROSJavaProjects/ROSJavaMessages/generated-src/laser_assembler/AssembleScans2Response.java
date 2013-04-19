package laser_assembler;

public interface AssembleScans2Response extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "laser_assembler/AssembleScans2Response";
  static final java.lang.String _DEFINITION = "# The point cloud holding the assembled clouds or scans. \n# This cloud is in the frame specified by the ~fixed_frame node parameter. \n# cloud is empty if no scans are received in the requested interval.\nsensor_msgs/PointCloud2 cloud";
  sensor_msgs.PointCloud2 getCloud();
  void setCloud(sensor_msgs.PointCloud2 value);
}
