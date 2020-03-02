package sandia_hand_msgs;

public interface RawTactile extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/RawTactile";
  static final java.lang.String _DEFINITION = "Header header\n\nuint16[] f0\nuint16[] f1\nuint16[] f2\nuint16[] f3\nuint16[] palm\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short[] getF0();
  void setF0(short[] value);
  short[] getF1();
  void setF1(short[] value);
  short[] getF2();
  void setF2(short[] value);
  short[] getF3();
  void setF3(short[] value);
  short[] getPalm();
  void setPalm(short[] value);
}
