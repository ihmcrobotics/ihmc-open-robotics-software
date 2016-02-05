package control_msgs;

public interface GripperCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "control_msgs/GripperCommand";
  static final java.lang.String _DEFINITION = "float64 position\nfloat64 max_effort\n";
  double getPosition();
  void setPosition(double value);
  double getMaxEffort();
  void setMaxEffort(double value);
}
