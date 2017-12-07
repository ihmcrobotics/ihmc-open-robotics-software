package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.rtps.DataConsumerParticipant;
import us.ihmc.robotDataLogger.rtps.LogProducerDisplay;
import us.ihmc.robotDataLogger.rtps.RTPSDebugRegistry;
import us.ihmc.robotDataLogger.rtps.VariableChangedProducer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoVariableClient
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
   

   private final LogProducerDisplay.LogSessionFilter[] sessionFilters;
   
   private ClientState state = ClientState.WAITING;

   private enum ClientState
   {
      WAITING, RUNNING, STOPPED
   }

   private static DataConsumerParticipant createParticipant()
   {
      try
      {
         return new DataConsumerParticipant("YoVariableClient");
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Start a new client while allowing the user to select a desired logging session
    * 
    * @param listener
    * @param filters
    */
   public YoVariableClient(YoVariablesUpdatedListener listener, LogProducerDisplay.LogSessionFilter... filters)
   {
      this(createParticipant(), listener, filters);
   }

   /**
    * Connect to an already selected log session
    * @param request
    * @param yoVariablesUpdatedListener
    */
   public YoVariableClient(DataConsumerParticipant participant, final YoVariablesUpdatedListener yoVariablesUpdatedListener)
   {
      this(participant, yoVariablesUpdatedListener, null);
   }

   private YoVariableClient(DataConsumerParticipant participant, final YoVariablesUpdatedListener yoVariablesUpdatedListener, LogProducerDisplay.LogSessionFilter[] filters)
   {
      this.dataConsumerParticipant = participant;
      this.yoVariablesUpdatedListener = yoVariablesUpdatedListener;
      this.sessionFilters = filters;
      if (yoVariablesUpdatedListener.changesVariables())
      {
         this.variableChangedProducer = new VariableChangedProducer();
      }
      else
      {
         this.variableChangedProducer = null;
      }

      this.handshakeParser = new IDLYoVariableHandshakeParser(HandshakeFileType.IDL_CDR);
      this.yoVariablesUpdatedListener.setYoVariableClient(this);
   }

   public String getServerName()
   {
      return serverName;
   }

   public void connectionClosed()
   {
      if (state == ClientState.RUNNING)
      {
         System.out.println("Disconnected, closing client.");
         dataConsumerParticipant.remove();
         yoVariablesUpdatedListener.disconnected();

         state = ClientState.STOPPED;
      }
   }

   public void start()
   {
      try
      {
         start(15000);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public synchronized void start(int timeout) throws IOException
   {
      if (state != ClientState.WAITING)
      {
         throw new RuntimeException("Client already started");
      }
      
      Announcement announcement = LogProducerDisplay.getAnnounceRequest(dataConsumerParticipant, sessionFilters);
      start(timeout, announcement);
   }
   
   public synchronized void start(int timeout, Announcement announcement) throws IOException
   {
      this.serverName = announcement.getNameAsString();

      if (state != ClientState.WAITING)
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

      yoVariablesUpdatedListener.start(logHandshake, handshakeParser);

      connectToSession(announcement);
      state = ClientState.RUNNING;
   }
   
   public void connectToSession(Announcement announcement) throws IOException
   {
      dataConsumerParticipant.createSession(announcement, handshakeParser, this, variableChangedProducer, yoVariablesUpdatedListener, yoVariablesUpdatedListener, debugRegistry);
   }

   public void requestStop()
   {
      dataConsumerParticipant.remove();
   }

   public synchronized boolean isRunning()
   {
      return state == ClientState.RUNNING;
   }

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

   public YoVariableRegistry getDebugRegistry()
   {
      return debugRegistry.getYoVariableRegistry();
   }

   public void receivedTimestampAndData(long timestamp)
   {
      yoVariablesUpdatedListener.receivedTimestampAndData(timestamp);
   }

}
