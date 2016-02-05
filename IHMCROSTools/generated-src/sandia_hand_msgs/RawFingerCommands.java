package sandia_hand_msgs;

public interface RawFingerCommands extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/RawFingerCommands";
  static final java.lang.String _DEFINITION = "int16[3] motor_targets\n";
  short[] getMotorTargets();
  void setMotorTargets(short[] value);
}
