package arm_navigation_msgs;

public interface CollisionMap extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/CollisionMap";
  static final java.lang.String _DEFINITION = "#header for interpreting box positions\nHeader header\n\n#boxes for use in collision testing\nOrientedBoundingBox[] boxes\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<arm_navigation_msgs.OrientedBoundingBox> getBoxes();
  void setBoxes(java.util.List<arm_navigation_msgs.OrientedBoundingBox> value);
}
