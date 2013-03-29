package gazebo;

public interface SetLinkStateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetLinkStateRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\ngazebo/LinkState link_state\n";
  gazebo.LinkState getLinkState();
  void setLinkState(gazebo.LinkState value);
}
