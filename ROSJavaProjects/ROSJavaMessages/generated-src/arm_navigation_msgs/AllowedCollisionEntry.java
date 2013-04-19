package arm_navigation_msgs;

public interface AllowedCollisionEntry extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/AllowedCollisionEntry";
  static final java.lang.String _DEFINITION = "# whether or not collision checking is enabled\nbool[] enabled\n";
  boolean[] getEnabled();
  void setEnabled(boolean[] value);
}
