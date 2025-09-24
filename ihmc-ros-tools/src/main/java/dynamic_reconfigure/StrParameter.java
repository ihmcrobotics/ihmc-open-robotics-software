package dynamic_reconfigure;

import org.ros.internal.message.Message;

public interface StrParameter extends Message {
   String _TYPE = "dynamic_reconfigure/StrParameter";
   String _DEFINITION = "string name\nstring value\n";

   String getName();

   void setName(String var1);

   String getValue();

   void setValue(String var1);
}