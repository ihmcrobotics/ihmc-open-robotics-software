package sandia_hand_msgs;

public interface SetFingerHomeRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/SetFingerHomeRequest";
  static final java.lang.String _DEFINITION = "uint8 finger_idx\n";
  byte getFingerIdx();
  void setFingerIdx(byte value);
}
