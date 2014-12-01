package us.ihmc.multicastLogDataProtocol.control;

import java.net.InetAddress;
import java.net.UnknownHostException;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.utilities.net.KryoObjectClient;

public class LogControlClient
{
   private final KryoObjectClient client;
   public LogControlClient(byte[] host, int port)
   {
      try
      {
         client = new KryoObjectClient(InetAddress.getByAddress(host), port, new LogControlClassList());
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public LogModelLoader getModelLoader()
   {
      return null;
   }
   
}
