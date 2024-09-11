package dynamic_reconfigure;

import org.ros.internal.message.Message;

public interface GroupState extends Message {
   String _TYPE = "dynamic_reconfigure/GroupState";
   String _DEFINITION = "string name\nbool state\nint32 id\nint32 parent\n";

   String getName();

   void setName(String var1);

   boolean getState();

   void setState(boolean var1);

   int getId();

   void setId(int var1);

   int getParent();

   void setParent(int var1);
}