package sandia_hand_msgs;

public interface RawMoboState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/RawMoboState";
  static final java.lang.String _DEFINITION = "uint32 mobo_time\nfloat32[4] finger_currents\nfloat32[3] logic_currents\nuint16[3] mobo_temp\nuint8 mobo_max_effort\n";
  int getMoboTime();
  void setMoboTime(int value);
  float[] getFingerCurrents();
  void setFingerCurrents(float[] value);
  float[] getLogicCurrents();
  void setLogicCurrents(float[] value);
  short[] getMoboTemp();
  void setMoboTemp(short[] value);
  byte getMoboMaxEffort();
  void setMoboMaxEffort(byte value);
}
