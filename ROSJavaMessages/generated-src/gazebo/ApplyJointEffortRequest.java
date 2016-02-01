package gazebo;

public interface ApplyJointEffortRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/ApplyJointEffortRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\n# set urdf joint effort\nstring joint_name           # joint to apply wrench (linear force and torque)\nfloat64 effort              # effort to apply\ntime start_time             # optional wrench application start time (seconds)\n                            # if start_time < current time, start as soon as possible\nduration duration           # optional duration of wrench application time (seconds)\n                            # if duration < 0, apply wrench continuously without end\n                            # if duration = 0, do nothing\n                            # if duration < step size, assume step size and\n                            #               display warning in status_message\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
  double getEffort();
  void setEffort(double value);
  org.ros.message.Time getStartTime();
  void setStartTime(org.ros.message.Time value);
  org.ros.message.Duration getDuration();
  void setDuration(org.ros.message.Duration value);
}
