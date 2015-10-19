package gazebo_msgs;

public interface ApplyJointEffort extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/ApplyJointEffort";
  static final java.lang.String _DEFINITION = "# set urdf joint effort\nstring joint_name           # joint to apply wrench (linear force and torque)\nfloat64 effort              # effort to apply\ntime start_time             # optional wrench application start time (seconds)\n                            # if start_time < current time, start as soon as possible\nduration duration           # optional duration of wrench application time (seconds)\n                            # if duration < 0, apply wrench continuously without end\n                            # if duration = 0, do nothing\n                            # if duration < step size, assume step size and\n                            #               display warning in status_message\n---\nbool success                # return true if effort application is successful\nstring status_message       # comments if available\n";
}
