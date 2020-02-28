package handle_msgs;

public interface HandleSensorsCalibrated extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "handle_msgs/HandleSensorsCalibrated";
  static final java.lang.String _DEFINITION = "# This is sensors of the HANDLE hand after calibration and data manipulation\n# published from the package sensors, by the sensors_publisher\n\n# not all the sensors were included, but only the one which were addressed at the moment\n\n# Currently only used for time stamp.  \nHeader header\n\n# The tactile array for each finger.  In units of kPa.\n# [F1, F2, F3]\n# Note there are 12 proximal and 10 distal sensors.\nFinger[3] fingerTactile\n\n# The tactile array for the palm.  In units of kPa.\nfloat32[48] palmTactile\n\n# The encoder on the F1 / F2 rotation.\n# Approx. 768 ticks to rotate the fingers 90 degrees.\nfloat32 fingerSpread\n\n# The proximal joint angle. Angle in radians\n# [F1, F2, F3]\nfloat32[3] proximalJointAngle\n\n# The finger distal joint flexture angle\n# [F1, F2, F3]\n# Note there are 4 readings on either side of the joint.\nFinger[3] distalJointAngle\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<handle_msgs.Finger> getFingerTactile();
  void setFingerTactile(java.util.List<handle_msgs.Finger> value);
  float[] getPalmTactile();
  void setPalmTactile(float[] value);
  float getFingerSpread();
  void setFingerSpread(float value);
  float[] getProximalJointAngle();
  void setProximalJointAngle(float[] value);
  java.util.List<handle_msgs.Finger> getDistalJointAngle();
  void setDistalJointAngle(java.util.List<handle_msgs.Finger> value);
}
