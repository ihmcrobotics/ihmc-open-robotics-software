package us.ihmc.robotDataLogger.websocket.client;

import java.io.IOException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import io.netty.buffer.ByteBuf;
import io.netty.util.CharsetUtil;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.interfaces.DataConsumer;
import us.ihmc.robotDataLogger.websocket.LogHTTPPaths;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;

public class WebsocketDataConsumer implements DataConsumer
{
   private HTTPDataServerConnection connection;
   

   public WebsocketDataConsumer(HTTPDataServerConnection initialConnection)
   {
      this.connection = initialConnection;
   }
   
   
   private ByteBuf getResource(String path, int timeout) throws IOException
   {
      if(!connection.isConnectected())
      {
         throw new IOException("Not connected");
      }
      
      Future<ByteBuf> resourceFuture = connection.requestResource(path);
      
      try
      {
         return resourceFuture.get(timeout, TimeUnit.MILLISECONDS);
      }
      catch (Exception e)
      {
         throw new IOException(e);
      }
      
   }
   
   
   @Override
   public byte[] getModelFile(Announcement announcement, int timeout) throws IOException
   {  
      ByteBuf model = getResource(LogHTTPPaths.model, timeout);
      byte[] retVal = new byte[model.readableBytes()];
      model.readBytes(retVal);

      return retVal;
   }

   @Override
   public byte[] getResourceZip(Announcement announcement, int timeout) throws IOException
   {
      ByteBuf resourceZip = getResource(LogHTTPPaths.resources, timeout);
      byte[] retVal = new byte[resourceZip.readableBytes()];
      resourceZip.readBytes(retVal);
      return retVal;
   }

   @Override
   public Handshake getHandshake(Announcement announcement, int timeout) throws IOException
   {
      ByteBuf handshake = getResource(LogHTTPPaths.resources, timeout);
      
      JSONSerializer<Handshake> serializer = new JSONSerializer<Handshake>(new HandshakePubSubType());
      return serializer.deserialize(handshake.toString(CharsetUtil.UTF_8));
   }

   @Override
   public void remove()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void sendClearLogRequest() throws IOException
   {
      // TODO Auto-generated method stub
      
   }

}
