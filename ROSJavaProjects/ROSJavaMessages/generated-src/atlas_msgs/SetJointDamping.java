package atlas_msgs;

public interface SetJointDamping extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/SetJointDamping";
  static final java.lang.String _DEFINITION = "# permissible values for mode\n\n# joint damping coefficient, per joint bounds specified in AtlasPlugin.cc\nfloat64[28] damping_coefficients\n\n---\nbool success\nstring status_message\n\n";
}
