package gazebo_msgs;

public interface GetLinkPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetLinkPropertiesRequest";
  static final java.lang.String _DEFINITION = "string link_name          # name of link\n                          # link names are prefixed by model name, e.g. pr2::base_link\n";
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
}
