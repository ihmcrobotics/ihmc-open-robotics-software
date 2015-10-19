package multisense_ros;

public interface RawLidarData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "multisense_ros/RawLidarData";
  static final java.lang.String _DEFINITION = "uint32 scan_count\ntime time_start\ntime time_end\nint32 angle_start\nint32 angle_end\nuint32[] distance\nuint32[] intensity\n";
  int getScanCount();
  void setScanCount(int value);
  org.ros.message.Time getTimeStart();
  void setTimeStart(org.ros.message.Time value);
  org.ros.message.Time getTimeEnd();
  void setTimeEnd(org.ros.message.Time value);
  int getAngleStart();
  void setAngleStart(int value);
  int getAngleEnd();
  void setAngleEnd(int value);
  int[] getDistance();
  void setDistance(int[] value);
  int[] getIntensity();
  void setIntensity(int[] value);
}
