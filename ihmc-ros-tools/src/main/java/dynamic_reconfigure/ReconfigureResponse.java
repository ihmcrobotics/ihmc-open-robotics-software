package dynamic_reconfigure;

import org.ros.internal.message.Message;

public interface ReconfigureResponse extends Message {
   String _TYPE = "dynamic_reconfigure/ReconfigureResponse";
   String _DEFINITION = "Config config";

   Config getConfig();

   void setConfig(Config var1);
}