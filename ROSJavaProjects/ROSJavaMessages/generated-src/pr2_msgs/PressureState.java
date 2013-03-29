package pr2_msgs;

public interface PressureState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/PressureState";
  static final java.lang.String _DEFINITION = "#Output from finger-tip pressure sensors on PR2\n#Higher number correspond to more pressure, but there is no explicit unit and you will have to calibrate for offset\n#The numbers reported are the raw values from the I2C hardware\n\nHeader header\nint16[] l_finger_tip\nint16[] r_finger_tip\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short[] getLFingerTip();
  void setLFingerTip(short[] value);
  short[] getRFingerTip();
  void setRFingerTip(short[] value);
}
