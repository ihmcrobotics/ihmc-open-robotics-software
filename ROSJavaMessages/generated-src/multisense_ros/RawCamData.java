package multisense_ros;

public interface RawCamData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "multisense_ros/RawCamData";
  static final java.lang.String _DEFINITION = "float32 frames_per_second\nfloat32 gain\nuint32  exposure_time\nuint32  frame_count\ntime    time_stamp\nuint32  angle\nuint16  width\nuint16  height\nuint8[] gray_scale_image\nuint16[] disparity_image\n";
  float getFramesPerSecond();
  void setFramesPerSecond(float value);
  float getGain();
  void setGain(float value);
  int getExposureTime();
  void setExposureTime(int value);
  int getFrameCount();
  void setFrameCount(int value);
  org.ros.message.Time getTimeStamp();
  void setTimeStamp(org.ros.message.Time value);
  int getAngle();
  void setAngle(int value);
  short getWidth();
  void setWidth(short value);
  short getHeight();
  void setHeight(short value);
  org.jboss.netty.buffer.ChannelBuffer getGrayScaleImage();
  void setGrayScaleImage(org.jboss.netty.buffer.ChannelBuffer value);
  short[] getDisparityImage();
  void setDisparityImage(short[] value);
}
