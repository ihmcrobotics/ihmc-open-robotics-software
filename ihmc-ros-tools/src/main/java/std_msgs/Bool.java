package std_msgs;

import org.ros.internal.message.Message;

public interface Bool extends Message {
   java.lang.String _TYPE = "std_msgs/Bool";
   java.lang.String _DEFINITION = "bool data";

   boolean getData();

   void setData(boolean var1);
}