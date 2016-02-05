package pr2_controllers_msgs;

public interface Pr2GripperCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_controllers_msgs/Pr2GripperCommand";
  static final java.lang.String _DEFINITION = "float64 position\nfloat64 max_effort\n";
  double getPosition();
  void setPosition(double value);
  double getMaxEffort();
  void setMaxEffort(double value);
}
