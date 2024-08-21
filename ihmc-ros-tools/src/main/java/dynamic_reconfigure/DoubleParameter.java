package dynamic_reconfigure;

import org.ros.internal.message.Message;

public interface DoubleParameter extends Message {
   String _TYPE = "dynamic_reconfigure/DoubleParameter";
   String _DEFINITION = "string name\nfloat64 value\n";

   String getName();

   void setName(String var1);

   double getValue();

   void setValue(double var1);
}