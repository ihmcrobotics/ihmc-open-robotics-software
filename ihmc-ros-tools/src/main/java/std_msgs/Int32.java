package std_msgs;

import org.ros.internal.message.Message;

public interface Int32 extends Message {
   java.lang.String _TYPE = "std_msgs/Int32";
   java.lang.String _DEFINITION = "int32 data";

   int getData();

   void setData(int var1);
}