package atlas_msgs;

public interface AtlasBehaviorPelvisServoParams extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorPelvisServoParams";
  static final java.lang.String _DEFINITION = "# mirrored from AtlasControlTypes.h\n\nfloat64 pelvis_height\nfloat64 pelvis_yaw\nfloat64 pelvis_lat\n";
  double getPelvisHeight();
  void setPelvisHeight(double value);
  double getPelvisYaw();
  void setPelvisYaw(double value);
  double getPelvisLat();
  void setPelvisLat(double value);
}
