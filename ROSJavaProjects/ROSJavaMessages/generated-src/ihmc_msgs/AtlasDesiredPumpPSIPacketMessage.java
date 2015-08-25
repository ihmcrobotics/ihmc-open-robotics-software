package ihmc_msgs;

public interface AtlasDesiredPumpPSIPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/AtlasDesiredPumpPSIPacketMessage";
  static final java.lang.String _DEFINITION = "## AtlasDesiredPumpPSIPacketMessage\r\n# Send a request to change the desired PSI of the Atlas hydraulic pump.\r\n\r\nint32 desired_pump_psi\r\n\r\n\r\n";
  int getDesiredPumpPsi();
  void setDesiredPumpPsi(int value);
}
