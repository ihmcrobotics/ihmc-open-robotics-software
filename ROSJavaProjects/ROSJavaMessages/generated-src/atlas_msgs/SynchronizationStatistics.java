package atlas_msgs;

public interface SynchronizationStatistics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/SynchronizationStatistics";
  static final java.lang.String _DEFINITION = "float64 delay_in_step         # instantaneous delay per simulation step, this must be less than delayMaxPerStep.\nfloat64 delay_in_window       # total delay in current window period.\nfloat64 delay_window_remain   # time left in current window period, before next budget reset.\n";
  double getDelayInStep();
  void setDelayInStep(double value);
  double getDelayInWindow();
  void setDelayInWindow(double value);
  double getDelayWindowRemain();
  void setDelayWindowRemain(double value);
}
