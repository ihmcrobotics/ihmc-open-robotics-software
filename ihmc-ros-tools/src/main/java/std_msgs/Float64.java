package std_msgs;

import org.ros.internal.message.Message;

public interface Float64 extends Message {
   java.lang.String _TYPE = "std_msgs/Float64";
   java.lang.String _DEFINITION = "float64 data";

   double getData();

   void setData(double var1);
}
