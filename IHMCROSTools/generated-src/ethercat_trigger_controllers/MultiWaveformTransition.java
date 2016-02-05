package ethercat_trigger_controllers;

public interface MultiWaveformTransition extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethercat_trigger_controllers/MultiWaveformTransition";
  static final java.lang.String _DEFINITION = "# Used to specify a transition in the SetMultiWaveform service.\n\nfloat64 time # Transition time after start of period.\nuint32 value # Value of the digital output after the transition time.\nstring topic # Topic to publish the transition timestamp to, or empty string if the transition should not be published.\n";
  double getTime();
  void setTime(double value);
  int getValue();
  void setValue(int value);
  java.lang.String getTopic();
  void setTopic(java.lang.String value);
}
