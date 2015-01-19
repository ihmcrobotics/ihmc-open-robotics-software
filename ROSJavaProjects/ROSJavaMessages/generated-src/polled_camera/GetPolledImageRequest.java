package polled_camera;

public interface GetPolledImageRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "polled_camera/GetPolledImageRequest";
  static final java.lang.String _DEFINITION = "# Namespace to publish response topics in. A polled camera driver node should publish:\n#   <response_namespace>/image_raw\n#   <response_namespace>/camera_info\nstring response_namespace\n\n# Region of interest, if supported by the camera.\nsensor_msgs/RegionOfInterest roi\n";
  java.lang.String getResponseNamespace();
  void setResponseNamespace(java.lang.String value);
  sensor_msgs.RegionOfInterest getRoi();
  void setRoi(sensor_msgs.RegionOfInterest value);
}
