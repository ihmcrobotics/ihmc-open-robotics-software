package multisense_ros;

import org.ros.internal.message.Message;
import org.ros.message.Time;

public interface StampedPps extends Message {
   String _TYPE = "multisense_ros/StampedPps";
   String _DEFINITION = "time     data\ntime     host_time\n";

   Time getData();

   void setData(Time var1);

   Time getHostTime();

   void setHostTime(Time var1);
}