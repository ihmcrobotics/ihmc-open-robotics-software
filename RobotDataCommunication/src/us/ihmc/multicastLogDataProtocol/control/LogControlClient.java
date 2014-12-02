package us.ihmc.multicastLogDataProtocol.control;

import java.net.InetAddress;
import java.net.UnknownHostException;

import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.robotDataCommunication.YoVariablesUpdatedListener;
import us.ihmc.utilities.net.KryoObjectClient;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectConsumer;

public class LogControlClient implements NetStateListener 
{
   private final KryoObjectClient client;
   private final YoVariableHandshakeParser handshakeParser;
   
   
   public LogControlClient(byte[] host, int port, YoVariablesUpdatedListener listener, String registryPrefix, boolean registerYoVariables)
   {
      try
      {
         client = new KryoObjectClient(InetAddress.getByAddress(host), port, new LogControlClassList());
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
      handshakeParser = new YoVariableHandshakeParser(listener, registryPrefix, registerYoVariables);      
      client.attachStateListener(this);
      client.attachListener(LogHandshake.class, new LogHandShakeConsumer());
   }
   
   @Override
   public void connected()
   {
      client.consumeObject(new HandshakeRequest());
   }

   @Override
   public void disconnected()
   {
      // TODO Auto-generated method stub
      
   }
   
   private class LogHandShakeConsumer implements ObjectConsumer<LogHandshake>
   {
      private boolean receivedHandshake;
      
      @Override
      public synchronized void consumeObject(LogHandshake object)
      {
         if(!receivedHandshake)
         {
            handshakeParser.parseFrom(object.protoShake);
         }
      }
      
   }
   
}
