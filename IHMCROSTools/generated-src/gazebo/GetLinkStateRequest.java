package gazebo;

public interface GetLinkStateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetLinkStateRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring link_name          # name of link\nstring reference_frame    # reference frame of returned information, must be a valid link\n                          # if empty, use inertial (gazebo world) frame\n";
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
  java.lang.String getReferenceFrame();
  void setReferenceFrame(java.lang.String value);
}
