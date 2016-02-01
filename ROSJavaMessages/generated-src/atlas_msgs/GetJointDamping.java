package atlas_msgs;

public interface GetJointDamping extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/GetJointDamping";
  static final java.lang.String _DEFINITION = "# permissible values for mode\n---\n# joint damping coefficient, and per joint bounds specified in AtlasPlugin.cc\nfloat64[28] damping_coefficients\nfloat64[28] damping_coefficients_min\nfloat64[28] damping_coefficients_max\nbool success\nstring status_message\n\n";
}
