package sensor_msgs;

public interface Joy extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/Joy";
  static final java.lang.String _DEFINITION = "# Reports the state of a joysticks axes and buttons.\nHeader header           # timestamp in the header is the time the data is received from the joystick\nfloat32[] axes          # the axes measurements from a joystick\nint32[] buttons         # the buttons measurements from a joystick \n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float[] getAxes();
  void setAxes(float[] value);
  int[] getButtons();
  void setButtons(int[] value);
}
