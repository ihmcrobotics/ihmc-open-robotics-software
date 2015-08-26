package ihmc_msgs;

public interface AtlasDesiredPumpPSIPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/AtlasDesiredPumpPSIPacketMessage";
  static final java.lang.String _DEFINITION = "## AtlasDesiredPumpPSIPacketMessage\n# Send a request to change the desired PSI of the Atlas hydraulic pump.\n\nint32 desired_pump_psi\n\nint64 unique_id\n\n\n";
  int getDesiredPumpPsi();
  void setDesiredPumpPsi(int value);
  long getUniqueId();
  void setUniqueId(long value);
}
