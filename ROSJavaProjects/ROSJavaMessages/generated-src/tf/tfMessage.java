package tf;

public interface tfMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "tf/tfMessage";
  static final java.lang.String _DEFINITION = "geometry_msgs/TransformStamped[] transforms\n";
  java.util.List<geometry_msgs.TransformStamped> getTransforms();
  void setTransforms(java.util.List<geometry_msgs.TransformStamped> value);
}
