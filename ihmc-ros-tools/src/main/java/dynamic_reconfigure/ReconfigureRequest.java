package dynamic_reconfigure;

import org.ros.internal.message.Message;

public interface ReconfigureRequest extends Message {
   String _TYPE = "dynamic_reconfigure/ReconfigureRequest";
   String _DEFINITION = "Config config\n";

   Config getConfig();

   void setConfig(Config var1);
}