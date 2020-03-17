package handle_msgs;

public interface CableTension extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "handle_msgs/CableTension";
  static final java.lang.String _DEFINITION = "# The cable tension in one finger of the HANDLE hand.\n\nfloat32 sensor1\nfloat32 sensor2\n";
  float getSensor1();
  void setSensor1(float value);
  float getSensor2();
  void setSensor2(float value);
}
