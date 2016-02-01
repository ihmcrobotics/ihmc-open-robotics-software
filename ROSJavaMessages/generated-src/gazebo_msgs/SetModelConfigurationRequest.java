package gazebo_msgs;

public interface SetModelConfigurationRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetModelConfigurationRequest";
  static final java.lang.String _DEFINITION = "# Set Gazebo Model pose and twist\nstring model_name           # model to set state (pose and twist)\nstring urdf_param_name      # parameter name that contains the urdf XML.\n\nstring[] joint_names        # list of joints to set positions.  if joint is not listed here, preserve current position.\nfloat64[] joint_positions   # set to this position.\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
  java.lang.String getUrdfParamName();
  void setUrdfParamName(java.lang.String value);
  java.util.List<java.lang.String> getJointNames();
  void setJointNames(java.util.List<java.lang.String> value);
  double[] getJointPositions();
  void setJointPositions(double[] value);
}
