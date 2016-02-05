package arm_navigation_msgs;

public interface OrientedBoundingBox extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/OrientedBoundingBox";
  static final java.lang.String _DEFINITION = "#the center of the box\ngeometry_msgs/Point32 center\n\n#the extents of the box, assuming the center is at the point\ngeometry_msgs/Point32 extents\n\n#the axis of the box\ngeometry_msgs/Point32 axis\n\n#the angle of rotation around the axis\nfloat32 angle\n";
  geometry_msgs.Point32 getCenter();
  void setCenter(geometry_msgs.Point32 value);
  geometry_msgs.Point32 getExtents();
  void setExtents(geometry_msgs.Point32 value);
  geometry_msgs.Point32 getAxis();
  void setAxis(geometry_msgs.Point32 value);
  float getAngle();
  void setAngle(float value);
}
