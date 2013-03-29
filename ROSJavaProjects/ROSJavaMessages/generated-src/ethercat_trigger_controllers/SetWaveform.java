package ethercat_trigger_controllers;

public interface SetWaveform extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethercat_trigger_controllers/SetWaveform";
  static final java.lang.String _DEFINITION = "float64 rep_rate\nfloat64 phase\nfloat64 duty_cycle\nint32 running\nint32 active_low\nint32 pulsed\n---\n";
}
