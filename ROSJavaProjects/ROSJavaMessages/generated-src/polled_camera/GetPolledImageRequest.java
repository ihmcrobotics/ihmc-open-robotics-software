package polled_camera;

public interface GetPolledImageRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "polled_camera/GetPolledImageRequest";
  static final java.lang.String _DEFINITION = "# Namespace to publish response topics in. A polled camera driver node\n# should publish:\n#   <response_namespace>/image_raw\n#   <response_namespace>/camera_info\nstring response_namespace\n\n# Timeout for attempting to capture data from the device. This does not\n# include latency from ROS communication, post-processing of raw camera\n# data, etc. A zero duration indicates no time limit.\nduration timeout\n\n# Binning settings, if supported by the camera.\nuint32 binning_x\nuint32 binning_y\n\n# Region of interest, if supported by the camera.\nsensor_msgs/RegionOfInterest roi\n";
  java.lang.String getResponseNamespace();
  void setResponseNamespace(java.lang.String value);
  org.ros.message.Duration getTimeout();
  void setTimeout(org.ros.message.Duration value);
  int getBinningX();
  void setBinningX(int value);
  int getBinningY();
  void setBinningY(int value);
  sensor_msgs.RegionOfInterest getRoi();
  void setRoi(sensor_msgs.RegionOfInterest value);
}
