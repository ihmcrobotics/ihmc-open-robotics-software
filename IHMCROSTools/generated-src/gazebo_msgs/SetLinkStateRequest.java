package gazebo_msgs;

public interface SetLinkStateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetLinkStateRequest";
  static final java.lang.String _DEFINITION = "gazebo_msgs/LinkState link_state\n";
  gazebo_msgs.LinkState getLinkState();
  void setLinkState(gazebo_msgs.LinkState value);
}
