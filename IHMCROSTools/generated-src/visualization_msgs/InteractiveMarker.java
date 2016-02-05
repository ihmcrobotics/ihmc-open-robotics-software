package visualization_msgs;

public interface InteractiveMarker extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "visualization_msgs/InteractiveMarker";
  static final java.lang.String _DEFINITION = "# Time/frame info.\n# If header.time is set to 0, the marker will be retransformed into\n# its frame on each timestep. You will receive the pose feedback\n# in the same frame.\n# Otherwise, you might receive feedback in a different frame.\n# For rviz, this will be the current \'fixed frame\' set by the user.\nHeader header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n\n# Short description (< 40 characters).\nstring description\n\n# Scale to be used for default controls (default=1).\nfloat32 scale\n\n# All menu and submenu entries associated with this marker.\nMenuEntry[] menu_entries\n\n# List of controls displayed for this marker.\nInteractiveMarkerControl[] controls\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getDescription();
  void setDescription(java.lang.String value);
  float getScale();
  void setScale(float value);
  java.util.List<visualization_msgs.MenuEntry> getMenuEntries();
  void setMenuEntries(java.util.List<visualization_msgs.MenuEntry> value);
  java.util.List<visualization_msgs.InteractiveMarkerControl> getControls();
  void setControls(java.util.List<visualization_msgs.InteractiveMarkerControl> value);
}
