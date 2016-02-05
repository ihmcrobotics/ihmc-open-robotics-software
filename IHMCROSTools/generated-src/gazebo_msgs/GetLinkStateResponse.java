package gazebo_msgs;

public interface GetLinkStateResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetLinkStateResponse";
  static final java.lang.String _DEFINITION = "gazebo_msgs/LinkState link_state\nbool success              # return true if get info is successful\nstring status_message     # comments if available";
  gazebo_msgs.LinkState getLinkState();
  void setLinkState(gazebo_msgs.LinkState value);
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}
