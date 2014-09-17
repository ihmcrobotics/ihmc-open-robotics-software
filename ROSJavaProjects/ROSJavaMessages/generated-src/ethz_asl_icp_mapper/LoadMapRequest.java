package ethz_asl_icp_mapper;

public interface LoadMapRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/LoadMapRequest";
  static final java.lang.String _DEFINITION = "std_msgs/String filename\n";
  std_msgs.String getFilename();
  void setFilename(std_msgs.String value);
}
