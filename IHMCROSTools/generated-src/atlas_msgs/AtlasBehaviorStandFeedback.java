package atlas_msgs;

public interface AtlasBehaviorStandFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorStandFeedback";
  static final java.lang.String _DEFINITION = "# mirrored from AtlasControlTypes.h\nuint32 status_flags    # use AtlasBeahviorStandFlags\n";
  int getStatusFlags();
  void setStatusFlags(int value);
}
