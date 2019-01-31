package us.ihmc.robotDataLogger.websocket.client;

import java.io.IOException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import io.netty.buffer.ByteBuf;
import io.netty.util.CharsetUtil;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.interfaces.DataConsumer;
import us.ihmc.robotDataLogger.listeners.ClearLogListener;
import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.rtps.RTPSDebugRegistry;
import us.ihmc.robotDataLogger.rtps.VariableChangedProducer;
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
      if (!connection.isConnected())
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
   public byte[] getModelFile(int timeout) throws IOException
   {
      ByteBuf model = getResource(LogHTTPPaths.model, timeout);
      byte[] retVal = new byte[model.readableBytes()];
      model.readBytes(retVal);

      return retVal;
   }

   @Override
   public byte[] getResourceZip(int timeout) throws IOException
   {
      ByteBuf resourceZip = getResource(LogHTTPPaths.resources, timeout);
      byte[] retVal = new byte[resourceZip.readableBytes()];
      resourceZip.readBytes(retVal);
      return retVal;
   }

   @Override
   public Handshake getHandshake(int timeout) throws IOException
   {
      ByteBuf handshake = getResource(LogHTTPPaths.handshake, timeout);

      JSONSerializer<Handshake> serializer = new JSONSerializer<Handshake>(new HandshakePubSubType());
      return serializer.deserialize(handshake.toString(CharsetUtil.UTF_8));
   }

   @Override
   public void sendClearLogRequest() throws IOException
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void startSession(IDLYoVariableHandshakeParser parser, YoVariableClientImplementation yoVariableClient,
                            VariableChangedProducer variableChangedProducer, TimestampListener timeStampListener, ClearLogListener clearLogListener,
                            RTPSDebugRegistry rtpsDebugRegistry)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isSessionActive()
   {
      return connection.isConnected();
   }

   @Override
   public void disconnectSession()
   {
      connection.close();
   }

   @Override
   public void close()
   {
      connection.close();
   }

   @Override
   public boolean isClosed()
   {
      return connection.isConnected();
   }

   @Override
   public boolean reconnect()
   {
      return false;
   }

}
