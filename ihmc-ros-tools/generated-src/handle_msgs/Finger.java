package handle_msgs;

public interface Finger extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "handle_msgs/Finger";
  static final java.lang.String _DEFINITION = "# This finger definition is used for different sensors in the HandleSensors \n# message type.  \n\nfloat32[] proximal\nfloat32[] distal\n\n";
  float[] getProximal();
  void setProximal(float[] value);
  float[] getDistal();
  void setDistal(float[] value);
}
