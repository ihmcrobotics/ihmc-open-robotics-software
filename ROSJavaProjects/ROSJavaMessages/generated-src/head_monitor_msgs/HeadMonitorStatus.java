package head_monitor_msgs;

public interface HeadMonitorStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "head_monitor_msgs/HeadMonitorStatus";
  static final java.lang.String _DEFINITION = "int32 status\n\nint32 IDLE = 1 \nint32 PREPLAN_SCAN = 2\nint32 PAUSED = 3\nint32 MONITOR_BEFORE_EXECUTION = 4\nint32 EXECUTING = 5\nint32 PAUSE_TIMEOUT = 6\nint32 COMPLETED = 7\nint32 PREEMPTED = 8\n";
  static final int IDLE = 1;
  static final int PREPLAN_SCAN = 2;
  static final int PAUSED = 3;
  static final int MONITOR_BEFORE_EXECUTION = 4;
  static final int EXECUTING = 5;
  static final int PAUSE_TIMEOUT = 6;
  static final int COMPLETED = 7;
  static final int PREEMPTED = 8;
  int getStatus();
  void setStatus(int value);
}
