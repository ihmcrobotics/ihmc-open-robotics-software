package gazebo;

public interface SetLinkState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetLinkState";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\ngazebo/LinkState link_state\n---\nbool success                # return true if set wrench successful\nstring status_message       # comments if available\n";
}
