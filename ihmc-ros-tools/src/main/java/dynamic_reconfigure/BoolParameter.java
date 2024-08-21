package dynamic_reconfigure;

import org.ros.internal.message.Message;

public interface BoolParameter extends Message {
   String _TYPE = "dynamic_reconfigure/BoolParameter";
   String _DEFINITION = "string name\nbool value\n";

   String getName();

   void setName(String var1);

   boolean getValue();

   void setValue(boolean var1);
}