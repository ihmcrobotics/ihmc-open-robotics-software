package arm_navigation_msgs;

public interface GetCollisionObjectsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetCollisionObjectsRequest";
  static final java.lang.String _DEFINITION = "#Whether or not to include the points in the collision map\n#if set to false, collision map in feedback will contain\n#no points\nbool include_points\n";
  boolean getIncludePoints();
  void setIncludePoints(boolean value);
}
