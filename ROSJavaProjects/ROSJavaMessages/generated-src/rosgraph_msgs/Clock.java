package rosgraph_msgs;

public interface Clock extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rosgraph_msgs/Clock";
  static final java.lang.String _DEFINITION = "# roslib/Clock is used for publishing simulated time in ROS. \n# This message simply communicates the current time.\n# For more information, see http://www.ros.org/wiki/Clock\ntime clock\n";
  org.ros.message.Time getClock();
  void setClock(org.ros.message.Time value);
}
