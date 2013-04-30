package atlas_msgs;

public interface AtlasBehaviorStepFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorStepFeedback";
  static final java.lang.String _DEFINITION = "# mirrored from AtlasControlTypes.h\nuint32 status_flags    # use AtlasBeahviorFeedback/status_flags\n";
  int getStatusFlags();
  void setStatusFlags(int value);
}
