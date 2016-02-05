package sandia_hand_msgs;

public interface RawPalmStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/RawPalmStatus";
  static final java.lang.String _DEFINITION = "uint32     palm_time\nint16[3]   palm_accel\nint16[3]   palm_gyro\nint16[3]   palm_mag\nuint16[7]  palm_temps\nuint16[32] palm_tactile\n";
  int getPalmTime();
  void setPalmTime(int value);
  short[] getPalmAccel();
  void setPalmAccel(short[] value);
  short[] getPalmGyro();
  void setPalmGyro(short[] value);
  short[] getPalmMag();
  void setPalmMag(short[] value);
  short[] getPalmTemps();
  void setPalmTemps(short[] value);
  short[] getPalmTactile();
  void setPalmTactile(short[] value);
}
