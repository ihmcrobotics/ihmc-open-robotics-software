package polled_camera;

public interface GetPolledImage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "polled_camera/GetPolledImage";
  static final java.lang.String _DEFINITION = "# Namespace to publish response topics in. A polled camera driver node should publish:\n#   <response_namespace>/image_raw\n#   <response_namespace>/camera_info\nstring response_namespace\n\n# Region of interest, if supported by the camera.\nsensor_msgs/RegionOfInterest roi\n---\n# Timestamp of the captured image. Can be matched against incoming sensor_msgs/Image header.\ntime stamp\n";
}
