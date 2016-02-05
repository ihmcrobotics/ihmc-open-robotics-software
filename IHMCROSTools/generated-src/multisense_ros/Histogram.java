package multisense_ros;

public interface Histogram extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "multisense_ros/Histogram";
  static final java.lang.String _DEFINITION = "int64    frame_count\ntime     time_stamp\nuint32   width\nuint32   height\nfloat32  gain\nfloat32  fps\nuint32   exposure_time\nuint32   channels\nuint32   bins\nuint32[] data\n";
  long getFrameCount();
  void setFrameCount(long value);
  org.ros.message.Time getTimeStamp();
  void setTimeStamp(org.ros.message.Time value);
  int getWidth();
  void setWidth(int value);
  int getHeight();
  void setHeight(int value);
  float getGain();
  void setGain(float value);
  float getFps();
  void setFps(float value);
  int getExposureTime();
  void setExposureTime(int value);
  int getChannels();
  void setChannels(int value);
  int getBins();
  void setBins(int value);
  int[] getData();
  void setData(int[] value);
}
