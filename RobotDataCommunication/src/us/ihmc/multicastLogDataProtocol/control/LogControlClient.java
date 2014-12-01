package us.ihmc.multicastLogDataProtocol.control;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.utilities.net.KryoObjectClient;

public class LogControlClient
{
   private final KryoObjectClient client;
   public LogControlClient(byte[] host, int port)
   {
      client = new KryoObjectClient(host, tcpPort, netClassList);
   }
   
   public LogModelLoader getModelLoader()
   {
      return null;
   }
   
}
