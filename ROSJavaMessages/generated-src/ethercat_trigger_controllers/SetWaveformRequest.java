package ethercat_trigger_controllers;

public interface SetWaveformRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethercat_trigger_controllers/SetWaveformRequest";
  static final java.lang.String _DEFINITION = "float64 rep_rate\nfloat64 phase\nfloat64 duty_cycle\nint32 running\nint32 active_low\nint32 pulsed\n";
  double getRepRate();
  void setRepRate(double value);
  double getPhase();
  void setPhase(double value);
  double getDutyCycle();
  void setDutyCycle(double value);
  int getRunning();
  void setRunning(int value);
  int getActiveLow();
  void setActiveLow(int value);
  int getPulsed();
  void setPulsed(int value);
}
