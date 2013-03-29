package visualization_msgs;

public interface InteractiveMarkerFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "visualization_msgs/InteractiveMarkerFeedback";
  static final java.lang.String _DEFINITION = "# Time/frame info.\nHeader header\n\n# Identifying string. Must be unique in the topic namespace.\nstring client_id\n\n# Feedback message sent back from the GUI, e.g.\n# when the status of an interactive marker was modified by the user.\n\n# Specifies which interactive marker and control this message refers to\nstring marker_name\nstring control_name\n\n# Type of the event\n# KEEP_ALIVE: sent while dragging to keep up control of the marker\n# MENU_SELECT: a menu entry has been selected\n# BUTTON_CLICK: a button control has been clicked\n# POSE_UPDATE: the pose has been changed using one of the controls\nuint8 KEEP_ALIVE = 0\nuint8 POSE_UPDATE = 1\nuint8 MENU_SELECT = 2\nuint8 BUTTON_CLICK = 3\n\nuint8 MOUSE_DOWN = 4\nuint8 MOUSE_UP = 5\n\nuint8 event_type\n\n# Current pose of the marker\n# Note: Has to be valid for all feedback types.\ngeometry_msgs/Pose pose\n\n# Contains the ID of the selected menu entry\n# Only valid for MENU_SELECT events.\nuint32 menu_entry_id\n\n# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point\n# may contain the 3 dimensional position of the event on the\n# control.  If it does, mouse_point_valid will be true.  mouse_point\n# will be relative to the frame listed in the header.\ngeometry_msgs/Point mouse_point\nbool mouse_point_valid\n";
  static final byte KEEP_ALIVE = 0;
  static final byte POSE_UPDATE = 1;
  static final byte MENU_SELECT = 2;
  static final byte BUTTON_CLICK = 3;
  static final byte MOUSE_DOWN = 4;
  static final byte MOUSE_UP = 5;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getClientId();
  void setClientId(java.lang.String value);
  java.lang.String getMarkerName();
  void setMarkerName(java.lang.String value);
  java.lang.String getControlName();
  void setControlName(java.lang.String value);
  byte getEventType();
  void setEventType(byte value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  int getMenuEntryId();
  void setMenuEntryId(int value);
  geometry_msgs.Point getMousePoint();
  void setMousePoint(geometry_msgs.Point value);
  boolean getMousePointValid();
  void setMousePointValid(boolean value);
}
