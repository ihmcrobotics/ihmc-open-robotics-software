package arm_navigation_msgs;

public interface AllowedCollisionMatrix extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/AllowedCollisionMatrix";
  static final java.lang.String _DEFINITION = "# the list of link names in the matrix\nstring[] link_names\n\n# the individual entries in the allowed collision matrix\n# symmetric, with same order as link_names\nAllowedCollisionEntry[] entries\n";
  java.util.List<java.lang.String> getLinkNames();
  void setLinkNames(java.util.List<java.lang.String> value);
  java.util.List<arm_navigation_msgs.AllowedCollisionEntry> getEntries();
  void setEntries(java.util.List<arm_navigation_msgs.AllowedCollisionEntry> value);
}
