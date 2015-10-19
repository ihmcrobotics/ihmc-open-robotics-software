package gazebo;

public interface GetLinkPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetLinkPropertiesRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring link_name          # name of link\n";
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
}
