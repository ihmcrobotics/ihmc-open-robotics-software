package atlas_msgs;

public interface AtlasBehaviorStandParams extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorStandParams";
  static final java.lang.String _DEFINITION = "# stand parameters (NOT YET IMPLEMENTED).\nbool use_desired_pelvis_height\nfloat64 desired_pelvis_height\nfloat64 desired_pelvis_yaw\nfloat64 desired_pelvis_lat\n# etc., more to come\n";
  boolean getUseDesiredPelvisHeight();
  void setUseDesiredPelvisHeight(boolean value);
  double getDesiredPelvisHeight();
  void setDesiredPelvisHeight(double value);
  double getDesiredPelvisYaw();
  void setDesiredPelvisYaw(double value);
  double getDesiredPelvisLat();
  void setDesiredPelvisLat(double value);
}
