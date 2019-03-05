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
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.interfaces.CommandListener;
import us.ihmc.robotDataLogger.interfaces.DataConsumer;
import us.ihmc.robotDataLogger.interfaces.VariableChangedProducer;
import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.HTTPDataServerPaths;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

public class WebsocketDataConsumer implements DataConsumer
{
   private final Object lock = new Object();
   private HTTPDataServerConnection connection;

   private WebsocketDataServerClient session;
   private boolean closed = false;
   
   private final int timeoutInMs;
   
   
   private IDLYoVariableHandshakeParser parser;
   private YoVariableClientImplementation yoVariableClient;
   private TimestampListener timestampListener;
   private DebugRegistry debugRegistry;

   public WebsocketDataConsumer(HTTPDataServerConnection initialConnection, int timeoutInMs)
   {
      this.connection = initialConnection;
      this.timeoutInMs = timeoutInMs;
   }

   private ByteBuf getResource(String path) throws IOException
   {
      synchronized (lock)
      {
         if (!connection.isConnected())
         {
            throw new IOException("Not connected");
         }

         Future<ByteBuf> resourceFuture = connection.requestResource(path);

         try
         {
            return resourceFuture.get(timeoutInMs, TimeUnit.MILLISECONDS);
         }
         catch (Exception e)
         {
            throw new IOException(e);
         }
      }

   }

   @Override
   public byte[] getModelFile() throws IOException
   {
      ByteBuf model = getResource(HTTPDataServerPaths.model);
      byte[] retVal = new byte[model.readableBytes()];
      model.readBytes(retVal);

      return retVal;
   }

   @Override
   public byte[] getResourceZip() throws IOException
   {
      ByteBuf resourceZip = getResource(HTTPDataServerPaths.resources);
      byte[] retVal = new byte[resourceZip.readableBytes()];
      resourceZip.readBytes(retVal);
      return retVal;
   }

   @Override
   public Handshake getHandshake() throws IOException
   {
      ByteBuf handshake = getResource(HTTPDataServerPaths.handshake);

      JSONSerializer<Handshake> serializer = new JSONSerializer<Handshake>(new HandshakePubSubType());
      return serializer.deserialize(handshake.toString(CharsetUtil.UTF_8));
   }


   @Override
   public void startSession(IDLYoVariableHandshakeParser parser, YoVariableClientImplementation yoVariableClient,
                            VariableChangedProducer variableChangedProducer, TimestampListener timeStampListener, CommandListener clearLogListener,
                            DebugRegistry debugRegistry)
         throws IOException
   {
      synchronized (lock)
      {
         if (!connection.isConnected())
         {
            throw new IOException("Not connected");
         }

         connection.take();
         this.parser = parser;
         this.timestampListener = timeStampListener;
         this.yoVariableClient = yoVariableClient;
         this.debugRegistry = debugRegistry;
         
         session = new WebsocketDataServerClient(connection, parser, timeStampListener, yoVariableClient, timeoutInMs, debugRegistry);
      }
   }

   @Override
   public boolean isSessionActive()
   {
      synchronized (lock)
      {
         if (session == null)
         {
            return false;
         }
         else
         {
            return session.isActive();
         }
      }
   }

   @Override
   public void disconnectSession()
   {
      synchronized (lock)
      {

         if (session == null)
         {
            throw new RuntimeException("Session not started");
         }

         session.close();
      }
   }

   @Override
   public void close()
   {
      synchronized (lock)
      {

         if (connection.isConnected())
         {
            connection.close();
         }

         if (session != null)
         {
            if (session.isActive())
            {
               session.close();
            }
         }

         closed = true;
      }
   }

   @Override
   public boolean isClosed()
   {
      synchronized (lock)
      {
         return closed;
      }
   }

   @Override
   public boolean reconnect() throws IOException
   {
      synchronized(lock)
      {
         if(session != null && session.isActive())
         {
            throw new RuntimeException("Session is still active");
         }

         try
         {
            HTTPDataServerDescription oldDescription = connection.getTarget();
            HTTPDataServerConnection newConnection = HTTPDataServerConnection.connect(oldDescription.getHost(), oldDescription.getPort());
            newConnection.close();
            
            Announcement announcement = newConnection.getAnnouncement();
            Announcement oldAnnouncement = connection.getAnnouncement();
            if(announcement.getReconnectKeyAsString().equals(oldAnnouncement.getReconnectKeyAsString()))
            {
               connection = newConnection;
               session = new WebsocketDataServerClient(connection, parser, timestampListener, yoVariableClient, timeoutInMs, debugRegistry);
               return true;
            }
            else
            {
               return false;
            }
         }
         catch(IOException e)
         {
            System.err.println(e.getMessage());
            return false;
         }   
      }
      
   }

   @Override
   public void writeVariableChangeRequest(int identifier, double valueAsDouble)
   {
      synchronized(lock)
      {
         if(session != null && session.isActive())
         {
            session.writeVariableChangeRequest(identifier, valueAsDouble);
         }
      }
   }

   @Override
   public void sendCommand(DataServerCommand command, int argument) throws IOException
   {
      synchronized(lock)
      {
         if(session != null && session.isActive())
         {
            session.sendCommand(command, argument);
         }
      }
   }

}
