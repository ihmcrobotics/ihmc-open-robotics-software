package multisense_ros;

public interface RawCamCal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "multisense_ros/RawCamCal";
  static final java.lang.String _DEFINITION = "float32[9]  left_M\nfloat32[8]  left_D\nfloat32[9]  left_R\nfloat32[12] left_P\nfloat32[9]  right_M\nfloat32[8]  right_D\nfloat32[9]  right_R\nfloat32[12] right_P\n";
  float[] getLeftM();
  void setLeftM(float[] value);
  float[] getLeftD();
  void setLeftD(float[] value);
  float[] getLeftR();
  void setLeftR(float[] value);
  float[] getLeftP();
  void setLeftP(float[] value);
  float[] getRightM();
  void setRightM(float[] value);
  float[] getRightD();
  void setRightD(float[] value);
  float[] getRightR();
  void setRightR(float[] value);
  float[] getRightP();
  void setRightP(float[] value);
}
