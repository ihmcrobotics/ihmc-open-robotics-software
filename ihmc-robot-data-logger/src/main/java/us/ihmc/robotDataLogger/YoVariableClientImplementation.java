package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.interfaces.DataConsumer;
import us.ihmc.robotDataLogger.rtps.RTPSDebugRegistry;
import us.ihmc.robotDataLogger.rtps.VariableChangedProducer;
import us.ihmc.robotDataLogger.websocket.client.WebsocketDataConsumer;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Client for the logger 
 * 
 * This is a general client for a logging sessions. A listener can be attached to provide desired functionality.
 * 
 * @author jesper
 */
public class YoVariableClientImplementation implements YoVariableClientInterface
{
   private String serverName;

   private final VariableChangedProducer variableChangedProducer;

   // Callback
   private final YoVariablesUpdatedListener yoVariablesUpdatedListener;

   // Internal values
   private final IDLYoVariableHandshakeParser handshakeParser;
   private final RTPSDebugRegistry debugRegistry = new RTPSDebugRegistry();
   
   private DataConsumer dataConsumer;

   YoVariableClientImplementation(final YoVariablesUpdatedListener yoVariablesUpdatedListener)
   {
      this.yoVariablesUpdatedListener = yoVariablesUpdatedListener;
      if (yoVariablesUpdatedListener.changesVariables())
      {
         this.variableChangedProducer = new VariableChangedProducer();
      }
      else
      {
         this.variableChangedProducer = null;
      }

      this.handshakeParser = new IDLYoVariableHandshakeParser(HandshakeFileType.IDL_CDR);
   }

   @Override
   public String getServerName()
   {
      return serverName;
   }

   /**
    * Callback function from the RegistryConsumer. 
    * 
    * Gets called when the connection is closed, either by timeout or by user request.
    * 
    */
   public void connectionClosed()
   {
      LogTools.info("Disconnected, closing client.");
      yoVariablesUpdatedListener.disconnected();
   }
   
   /**
    * Start a new session and request the handshake and robot model.
    * 
    * This can only be called once for a client. To restart, use reconnect()
    * 
    * @param timeout
    * @param connection
    * @throws IOException
    */
   public synchronized void start(int timeout, HTTPDataServerConnection connection) throws IOException
   {
      if (dataConsumer != null)
      {
         throw new RuntimeException("Client already started");
      }
      
      Announcement announcement = connection.getAnnouncement();
      
      this.dataConsumer = new WebsocketDataConsumer(connection);      
      this.serverName = connection.getAnnouncement().getNameAsString();
      
      LogTools.info("Requesting handshake");
      Handshake handshake = dataConsumer.getHandshake(timeout);

      handshakeParser.parseFrom(handshake);

      LogHandshake logHandshake = new LogHandshake();
      logHandshake.setHandshake(handshake);
      if (announcement.getModelFileDescription().getHasModel())
      {
         logHandshake.setModelName(announcement.getModelFileDescription().getNameAsString());
         LogTools.info("Requesting model file");
         logHandshake.setModel(dataConsumer.getModelFile(timeout));
         logHandshake.setModelLoaderClass(announcement.getModelFileDescription().getModelLoaderClassAsString());
         logHandshake.setResourceDirectories(announcement.getModelFileDescription().getResourceDirectories().toStringArray());
         if (announcement.getModelFileDescription().getHasResourceZip())
         {
            LogTools.info("Requesting resource bundle");
            logHandshake.setResourceZip(dataConsumer.getResourceZip(timeout));
         }
         LogTools.info("Received model");

      }

      if(variableChangedProducer != null)
      {
         variableChangedProducer.startVariableChangedProducers(handshakeParser.getYoVariablesList());
      }
      yoVariablesUpdatedListener.start(this, logHandshake, handshakeParser);

      
      connectToSession();
   }
   
   /**
    * Internal function to connect to a session
    * 
    * Throws a runtimeexception if you are already connected or when the client has closed to connection
    * 
    * @param announcement
    * @throws IOException
    */
   void connectToSession() throws IOException
   {
      if(dataConsumer.isSessionActive())
      {
         throw new RuntimeException("Client already connected");
      }
      if(dataConsumer.isClosed())
      {
         throw new RuntimeException("Client has closed completely");
      }
      dataConsumer.startSession(handshakeParser, this, variableChangedProducer, yoVariablesUpdatedListener, yoVariablesUpdatedListener, debugRegistry);
   }

   /**
    * Stops the client completely. 
    * 
    * The participant leaves the domain and a reconnect is not possible.
    */
   @Override
   public synchronized void stop()
   {
      if(dataConsumer == null)
      {
         throw new RuntimeException("Session not started");
      }
      dataConsumer.close();
   }

   /**
    * Broadcast a clear log request for the current session
    * 
    * If no session is available, this request gets silently ignored.
    */
   @Override
   public void sendClearLogRequest()
   {
      try
      {
         dataConsumer.sendClearLogRequest();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   /**
    * 
    * @return YoVariableRegistry with debug variables for this instance of the YoVariableClient
    */
   @Override
   public YoVariableRegistry getDebugRegistry()
   {
      return debugRegistry.getYoVariableRegistry();
   }

   /**
    * Callback from the timestamp topic. Gets called immediately when a new timestamp has arrived from the logger.
    * 
    * @param timestamp
    */
   public void receivedTimestampAndData(long timestamp)
   {
      yoVariablesUpdatedListener.receivedTimestampAndData(timestamp);
   }

   @Override
   public void disconnect()
   {
      dataConsumer.disconnectSession();
   }

   @Override
   public synchronized boolean reconnect() throws IOException
   {
      if(dataConsumer == null)
      {
         throw new RuntimeException("Session not started");
      }
      
      return dataConsumer.reconnect();
      
      
//      Announcement newAnnouncement = dataConsumerParticipant.getReconnectableSession(handshakeAnnouncement);
//      if(newAnnouncement == null)
//      {
//         return false;
//      }
//      else
//      {
//         connectToSession(newAnnouncement);
//         return true;
//      }
   }

   public void connected()
   {
      yoVariablesUpdatedListener.connected();
   }
}
