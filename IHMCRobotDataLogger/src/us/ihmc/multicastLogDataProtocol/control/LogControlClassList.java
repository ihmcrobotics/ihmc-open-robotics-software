package us.ihmc.multicastLogDataProtocol.control;

import us.ihmc.communication.net.NetClassList;

public class LogControlClassList extends NetClassList
{
   public LogControlClassList()
   {
      registerPacketClass(LogHandshake.class);
      registerPacketField(byte[].class);
      registerPacketClass(VariableChangeRequest.class);
      
      registerPacketField(String[].class);
      
      registerPacketClass(ClearLogRequest.class);
   }
}
