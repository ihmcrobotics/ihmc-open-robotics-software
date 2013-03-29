package laser_assembler;

public interface AssembleScans extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "laser_assembler/AssembleScans";
  static final java.lang.String _DEFINITION = "# The time interval on which we want to aggregate scans\ntime begin\n# The end of the interval on which we want to assemble scans or clouds\ntime end\n---\n# The point cloud holding the assembled clouds or scans. \n# This cloud is in the frame specified by the ~fixed_frame node parameter. \n# cloud is empty if no scans are received in the requested interval.\nsensor_msgs/PointCloud cloud\n";
}
