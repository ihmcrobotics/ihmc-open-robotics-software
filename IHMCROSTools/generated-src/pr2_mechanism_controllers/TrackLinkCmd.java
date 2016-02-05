package pr2_mechanism_controllers;

public interface TrackLinkCmd extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_controllers/TrackLinkCmd";
  static final java.lang.String _DEFINITION = "int8 enable\nstring link_name\ngeometry_msgs/Point point\n";
  byte getEnable();
  void setEnable(byte value);
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
  geometry_msgs.Point getPoint();
  void setPoint(geometry_msgs.Point value);
}
