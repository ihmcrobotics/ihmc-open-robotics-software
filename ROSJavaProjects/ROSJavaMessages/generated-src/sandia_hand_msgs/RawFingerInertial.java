package sandia_hand_msgs;

public interface RawFingerInertial extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/RawFingerInertial";
  static final java.lang.String _DEFINITION = "uint16[3] mm_accel\nuint16[3] pp_accel\nuint16[3] dp_accel\nuint16[3] mm_mag\nuint16[3] pp_mag\nuint16[3] dp_mag\n";
  short[] getMmAccel();
  void setMmAccel(short[] value);
  short[] getPpAccel();
  void setPpAccel(short[] value);
  short[] getDpAccel();
  void setDpAccel(short[] value);
  short[] getMmMag();
  void setMmMag(short[] value);
  short[] getPpMag();
  void setPpMag(short[] value);
  short[] getDpMag();
  void setDpMag(short[] value);
}
