package atlas_msgs;

public interface ResetControlsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/ResetControlsRequest";
  static final java.lang.String _DEFINITION = "atlas_msgs/AtlasCommand atlas_command\n";
  atlas_msgs.AtlasCommand getAtlasCommand();
  void setAtlasCommand(atlas_msgs.AtlasCommand value);
}
