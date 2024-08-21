package std_msgs;

import org.ros.internal.message.Message;

public interface Time extends Message {
   java.lang.String _TYPE = "std_msgs/Time";
   java.lang.String _DEFINITION = "time data\n";

   org.ros.message.Time getData();

   void setData(org.ros.message.Time var1);
}