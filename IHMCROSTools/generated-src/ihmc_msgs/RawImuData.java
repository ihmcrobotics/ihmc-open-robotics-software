package ihmc_msgs;

public interface RawImuData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/RawImuData";
  static final java.lang.String _DEFINITION = "uint64 imu_timestamp # timestamp for imu data in microseconds\nuint64 packet_count\n\nfloat64 dax # delta angle (radians) in the frame of the IMU\nfloat64 day\nfloat64 daz\n\nfloat64 ddx # linear acceleration (m/s^2) in the frame of the IMU\nfloat64 ddy\nfloat64 ddz\n";
  long getImuTimestamp();
  void setImuTimestamp(long value);
  long getPacketCount();
  void setPacketCount(long value);
  double getDax();
  void setDax(double value);
  double getDay();
  void setDay(double value);
  double getDaz();
  void setDaz(double value);
  double getDdx();
  void setDdx(double value);
  double getDdy();
  void setDdy(double value);
  double getDdz();
  void setDdz(double value);
}
