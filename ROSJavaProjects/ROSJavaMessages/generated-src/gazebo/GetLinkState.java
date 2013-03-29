package gazebo;

public interface GetLinkState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetLinkState";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring link_name          # name of link\nstring reference_frame    # reference frame of returned information, must be a valid link\n                          # if empty, use inertial (gazebo world) frame\n---\ngazebo/LinkState link_state\nbool success              # return true if get info is successful\nstring status_message     # comments if available\n";
}
