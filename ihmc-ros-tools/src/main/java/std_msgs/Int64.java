package std_msgs;

import org.ros.internal.message.Message;

public interface Int64 extends Message {
   java.lang.String _TYPE = "std_msgs/Int64";
   java.lang.String _DEFINITION = "int64 data";

   long getData();

   void setData(long var1);
}