package ethercat_trigger_controllers;

public interface MultiWaveform extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethercat_trigger_controllers/MultiWaveform";
  static final java.lang.String _DEFINITION = "# Transitions will occur at k * period + zero_offset + transitions[j].time, where j and\n# k are integers.\n\nfloat64 period # Period of the waveform in seconds.\nfloat64 zero_offset # Time corresponding to a time of 0 in times[] in seconds\nMultiWaveformTransition[] transitions # Transitions in the waveform. Transition times should be in increasing order, and be between 0 (inclusive) and period (exclusive)\n";
  double getPeriod();
  void setPeriod(double value);
  double getZeroOffset();
  void setZeroOffset(double value);
  java.util.List<ethercat_trigger_controllers.MultiWaveformTransition> getTransitions();
  void setTransitions(java.util.List<ethercat_trigger_controllers.MultiWaveformTransition> value);
}
