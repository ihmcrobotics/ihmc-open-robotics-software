package multisense_ros;

public interface RawCamConfig extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "multisense_ros/RawCamConfig";
  static final java.lang.String _DEFINITION = "uint16  width\nuint16  height\nfloat32 frames_per_second\nfloat32 gain\nuint32  exposure_time\nfloat32 fx\nfloat32 fy\nfloat32 cx\nfloat32 cy\nfloat32 tx\nfloat32 ty\nfloat32 tz\nfloat32 roll\nfloat32 pitch\nfloat32 yaw\n";
  short getWidth();
  void setWidth(short value);
  short getHeight();
  void setHeight(short value);
  float getFramesPerSecond();
  void setFramesPerSecond(float value);
  float getGain();
  void setGain(float value);
  int getExposureTime();
  void setExposureTime(int value);
  float getFx();
  void setFx(float value);
  float getFy();
  void setFy(float value);
  float getCx();
  void setCx(float value);
  float getCy();
  void setCy(float value);
  float getTx();
  void setTx(float value);
  float getTy();
  void setTy(float value);
  float getTz();
  void setTz(float value);
  float getRoll();
  void setRoll(float value);
  float getPitch();
  void setPitch(float value);
  float getYaw();
  void setYaw(float value);
}
