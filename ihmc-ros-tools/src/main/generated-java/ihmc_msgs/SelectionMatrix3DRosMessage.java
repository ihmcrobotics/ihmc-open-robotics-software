package ihmc_msgs;

public interface SelectionMatrix3DRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SelectionMatrix3DRosMessage";
  static final java.lang.String _DEFINITION = "## SelectionMatrix3DRosMessage\n# \n\n# The Id of the reference frame defining the selection frame. When selecting the axes of interest,\n# these axes refer to the selection frame axes. This frame is optional. It is preferable to provide it\n# when possible, but when it is absent, i.e. equal to {@code 0L}, the selection matrix will then be\n# generated regardless to what frame is it used in.\nint64 selection_frame_id\n\n# Specifies whether the x-axis of the selection frame is an axis of interest.\nbool x_selected\n\n# Specifies whether the y-axis of the selection frame is an axis of interest.\nbool y_selected\n\n# Specifies whether the z-axis of the selection frame is an axis of interest.\nbool z_selected\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  long getSelectionFrameId();
  void setSelectionFrameId(long value);
  boolean getXSelected();
  void setXSelected(boolean value);
  boolean getYSelected();
  void setYSelected(boolean value);
  boolean getZSelected();
  void setZSelected(boolean value);
  long getUniqueId();
  void setUniqueId(long value);
}
