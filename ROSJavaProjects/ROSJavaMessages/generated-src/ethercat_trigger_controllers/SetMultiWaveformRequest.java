package ethercat_trigger_controllers;

public interface SetMultiWaveformRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethercat_trigger_controllers/SetMultiWaveformRequest";
  static final java.lang.String _DEFINITION = "MultiWaveform waveform\n";
  ethercat_trigger_controllers.MultiWaveform getWaveform();
  void setWaveform(ethercat_trigger_controllers.MultiWaveform value);
}
