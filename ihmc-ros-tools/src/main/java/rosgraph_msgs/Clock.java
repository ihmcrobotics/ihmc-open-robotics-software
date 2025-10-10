package rosgraph_msgs;

import org.ros.internal.message.Message;
import org.ros.message.Time;

public interface Clock extends Message {
   String _TYPE = "rosgraph_msgs/Clock";
   String _DEFINITION = "# roslib/Clock is used for publishing simulated time in ROS. \n# This message simply communicates the current time.\n# For more information, see http://www.ros.org/wiki/Clock\ntime clock\n";

   Time getClock();

   void setClock(Time var1);
}