package atlas_msgs;

public interface AtlasBehaviorManipulateParams extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorManipulateParams";
  static final java.lang.String _DEFINITION = "# mirrored from AtlasControlTypes.h\nbool use_desired\natlas_msgs/AtlasBehaviorPelvisServoParams desired\nbool use_demo_mode\n";
  boolean getUseDesired();
  void setUseDesired(boolean value);
  atlas_msgs.AtlasBehaviorPelvisServoParams getDesired();
  void setDesired(atlas_msgs.AtlasBehaviorPelvisServoParams value);
  boolean getUseDemoMode();
  void setUseDemoMode(boolean value);
}
