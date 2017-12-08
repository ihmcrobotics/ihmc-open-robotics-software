package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.rtps.DataConsumerParticipant;
import us.ihmc.robotDataLogger.rtps.RTPSDebugRegistry;
import us.ihmc.robotDataLogger.rtps.VariableChangedProducer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Client for the logger 
 * 
 * This is a general client for a logging sessions. A listener can be attached to provide desired functionality.
 * 
 * @author jesper
 *
 */
public class YoVariableClientImplementation implements YoVariableClientInterface
{
   private String serverName;

   //DDS
   private final DataConsumerParticipant dataConsumerParticipant;
   private final VariableChangedProducer variableChangedProducer;

   // Callback
   private final YoVariablesUpdatedListener yoVariablesUpdatedListener;

   // Internal values
   private final IDLYoVariableHandshakeParser handshakeParser;
   private final RTPSDebugRegistry debugRegistry = new RTPSDebugRegistry();
   private Announcement handshakeAnnouncement;

   YoVariableClientImplementation(DataConsumerParticipant participant, final YoVariablesUpdatedListener yoVariablesUpdatedListener)
   {
      this.dataConsumerParticipant = participant;
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
      System.out.println("Disconnected, closing client.");
      dataConsumerParticipant.disconnectSession();
      yoVariablesUpdatedListener.disconnected();
   }
   
   /**
    * Start a new session and request the handshake and robot model.
    * 
    * This can only be called once for a client. To restart, use reconnect()
    * 
    * @param timeout
    * @param announcement
    * @throws IOException
    */
   public synchronized void start(int timeout, Announcement announcement) throws IOException
   {
      this.serverName = announcement.getNameAsString();

      if (handshakeAnnouncement != null)
      {
         throw new RuntimeException("Client already started");
      }
      
      System.out.println("Requesting handshake");
      Handshake handshake = dataConsumerParticipant.getHandshake(announcement, timeout);

      handshakeParser.parseFrom(handshake);

      LogHandshake logHandshake = new LogHandshake();
      logHandshake.setHandshake(handshake);
      if (announcement.getModelFileDescription().getHasModel())
      {
         logHandshake.setModelName(announcement.getModelFileDescription().getNameAsString());
         System.out.println("Requesting model file");
         logHandshake.setModel(dataConsumerParticipant.getModelFile(announcement, timeout));
         logHandshake.setModelLoaderClass(announcement.getModelFileDescription().getModelLoaderClassAsString());
         logHandshake.setResourceDirectories(announcement.getModelFileDescription().getResourceDirectories().toStringArray());
         if (announcement.getModelFileDescription().getHasResourceZip())
         {
            System.out.println("Requesting resource bundle");
            logHandshake.setResourceZip(dataConsumerParticipant.getResourceZip(announcement, timeout));
         }
         System.out.println("Received model");

      }

      if(variableChangedProducer != null)
      {
         variableChangedProducer.startVariableChangedProducers(handshakeParser.getYoVariablesList());
      }
      yoVariablesUpdatedListener.start(this, logHandshake, handshakeParser);

      handshakeAnnouncement = announcement;
      
      connectToSession(announcement);
   }
   
   /**
    * Internal function to connect to a session
    * 
    * Throws a runtimeexception if you are already connected or when the client has closed to connection
    * 
    * @param announcement
    * @throws IOException
    */
   void connectToSession(Announcement announcement) throws IOException
   {
      if(dataConsumerParticipant.isSessionActive())
      {
         throw new RuntimeException("Client already connected");
      }
      if(!dataConsumerParticipant.isConnectedToDomain())
      {
         throw new RuntimeException("Client has closed completly");
      }
      dataConsumerParticipant.createSession(announcement, handshakeParser, this, variableChangedProducer, yoVariablesUpdatedListener, yoVariablesUpdatedListener, debugRegistry);
   }

   /**
    * Stops the client completely. 
    * 
    * The participant leaves the domain and a reconnect is not possible.
    */
   @Override
   public synchronized void stop()
   {
      dataConsumerParticipant.remove();
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
         dataConsumerParticipant.sendClearLogRequest();
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
      dataConsumerParticipant.disconnectSession();
   }

   @Override
   public synchronized boolean reconnect() throws IOException
   {
      Announcement newAnnouncement = dataConsumerParticipant.getReconnectableSession(handshakeAnnouncement);
      if(newAnnouncement == null)
      {
         return false;
      }
      else
      {
         connectToSession(newAnnouncement);
         return true;
      }
   }

   public void connected()
   {
      yoVariablesUpdatedListener.connected();
   }

}
