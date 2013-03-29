package tf2_msgs;

public interface TFMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "tf2_msgs/TFMessage";
  static final java.lang.String _DEFINITION = "geometry_msgs/TransformStamped[] transforms\n";
  java.util.List<geometry_msgs.TransformStamped> getTransforms();
  void setTransforms(java.util.List<geometry_msgs.TransformStamped> value);
}
