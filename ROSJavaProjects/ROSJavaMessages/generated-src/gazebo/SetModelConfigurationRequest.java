package gazebo;

public interface SetModelConfigurationRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetModelConfigurationRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\n# Set Gazebo Model pose and twist\nstring model_name           # model to set state (pose and twist)\nstring test_urdf_param_name # parameter name that contains the urdf XML. (this is deprecated, new msg has no test_prefix.)\n\nstring[] joint_names        # list of joints to set positions.  if joint is not listed here, preserve current position.\nfloat64[] joint_positions   # set to this position.\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
  java.lang.String getTestUrdfParamName();
  void setTestUrdfParamName(java.lang.String value);
  java.util.List<java.lang.String> getJointNames();
  void setJointNames(java.util.List<java.lang.String> value);
  double[] getJointPositions();
  void setJointPositions(double[] value);
}
