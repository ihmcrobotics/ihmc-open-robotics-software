package dynamic_reconfigure;

public interface SensorLevels extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/SensorLevels";
  static final java.lang.String _DEFINITION = "# This message is deprecated, please use driver_base/SensorLevels instead.\n\nbyte RECONFIGURE_CLOSE = 3  # Parameters that need a sensor to be stopped completely when changed\nbyte RECONFIGURE_STOP = 1  # Parameters that need a sensor to stop streaming when changed\nbyte RECONFIGURE_RUNNING = 0 # Parameters that can be changed while a sensor is streaming\n";
  static final byte RECONFIGURE_CLOSE = 3;
  static final byte RECONFIGURE_STOP = 1;
  static final byte RECONFIGURE_RUNNING = 0;
}
