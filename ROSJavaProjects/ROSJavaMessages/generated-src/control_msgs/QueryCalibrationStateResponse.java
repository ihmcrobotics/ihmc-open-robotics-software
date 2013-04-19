package control_msgs;

public interface QueryCalibrationStateResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "control_msgs/QueryCalibrationStateResponse";
  static final java.lang.String _DEFINITION = "bool is_calibrated";
  boolean getIsCalibrated();
  void setIsCalibrated(boolean value);
}
