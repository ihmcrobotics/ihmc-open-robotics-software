package ihmc_msgs;

public interface BatchRawImuData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/BatchRawImuData";
  static final java.lang.String _DEFINITION = "Header header\nRawImuData[] data\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<ihmc_msgs.RawImuData> getData();
  void setData(java.util.List<ihmc_msgs.RawImuData> value);
}
