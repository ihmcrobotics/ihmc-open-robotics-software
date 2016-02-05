package polled_camera;

public interface GetPolledImage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "polled_camera/GetPolledImage";
  static final java.lang.String _DEFINITION = "# Namespace to publish response topics in. A polled camera driver node\n# should publish:\n#   <response_namespace>/image_raw\n#   <response_namespace>/camera_info\nstring response_namespace\n\n# Timeout for attempting to capture data from the device. This does not\n# include latency from ROS communication, post-processing of raw camera\n# data, etc. A zero duration indicates no time limit.\nduration timeout\n\n# Binning settings, if supported by the camera.\nuint32 binning_x\nuint32 binning_y\n\n# Region of interest, if supported by the camera.\nsensor_msgs/RegionOfInterest roi\n---\nbool success          # Could the image be captured?\nstring status_message # Error message in case of failure\ntime stamp            # Timestamp of the captured image. Can be matched\n                      # against incoming sensor_msgs/Image header.\n";
}
