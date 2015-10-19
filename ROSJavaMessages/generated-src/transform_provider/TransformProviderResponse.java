package transform_provider;

public interface TransformProviderResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "transform_provider/TransformProviderResponse";
  static final java.lang.String _DEFINITION = "geometry_msgs/TransformStamped transform";
  geometry_msgs.TransformStamped getTransform();
  void setTransform(geometry_msgs.TransformStamped value);
}
