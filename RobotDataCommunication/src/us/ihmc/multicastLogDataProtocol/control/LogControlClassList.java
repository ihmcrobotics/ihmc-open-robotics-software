package us.ihmc.multicastLogDataProtocol.control;

import us.ihmc.utilities.net.NetClassList;

public class LogControlClassList extends NetClassList
{
   public LogControlClassList()
   {
      registerPacketClass(LogHandshake.class);
      registerPacketField(byte[].class);
   }
}
