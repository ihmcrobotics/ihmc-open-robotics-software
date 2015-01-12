package sri_drc_hand;

public interface Status extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sri_drc_hand/Status";
  static final java.lang.String _DEFINITION = "# The status of each finger.\ntime stamp\nfloat32[] proximal_joint_position\nfloat32[] medial_joint_position\nfloat32[] distal_joint_position\nfloat32[] force\nfloat32[] current\n\nuint16 packet_counter\nuint32 timestamp\n# uses values from Command.msg\nuint8 mode\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  float[] getProximalJointPosition();
  void setProximalJointPosition(float[] value);
  float[] getMedialJointPosition();
  void setMedialJointPosition(float[] value);
  float[] getDistalJointPosition();
  void setDistalJointPosition(float[] value);
  float[] getForce();
  void setForce(float[] value);
  float[] getCurrent();
  void setCurrent(float[] value);
  short getPacketCounter();
  void setPacketCounter(short value);
  int getTimestamp();
  void setTimestamp(int value);
  byte getMode();
  void setMode(byte value);
}
