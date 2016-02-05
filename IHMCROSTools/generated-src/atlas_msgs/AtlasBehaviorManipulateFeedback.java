package atlas_msgs;

public interface AtlasBehaviorManipulateFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorManipulateFeedback";
  static final java.lang.String _DEFINITION = "# mirrored from AtlasControlTypes.h\nuint32 status_flags    # use AtlasBeahviorManipulateFlags\natlas_msgs/AtlasBehaviorPelvisServoParams clamped\n";
  int getStatusFlags();
  void setStatusFlags(int value);
  atlas_msgs.AtlasBehaviorPelvisServoParams getClamped();
  void setClamped(atlas_msgs.AtlasBehaviorPelvisServoParams value);
}
