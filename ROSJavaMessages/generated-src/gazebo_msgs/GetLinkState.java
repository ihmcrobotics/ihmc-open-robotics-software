package gazebo_msgs;

public interface GetLinkState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetLinkState";
  static final java.lang.String _DEFINITION = "string link_name          # name of link\n                          # link names are prefixed by model name, e.g. pr2::base_link\nstring reference_frame    # reference frame of returned information, must be a valid link\n                          # if empty, use inertial (gazebo world) frame\n                          # reference_frame names are prefixed by model name, e.g. pr2::base_link\n---\ngazebo_msgs/LinkState link_state\nbool success              # return true if get info is successful\nstring status_message     # comments if available\n";
}
