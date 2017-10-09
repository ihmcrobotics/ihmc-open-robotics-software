package us.ihmc.robotDataLogger;

import java.io.IOException;
import java.util.List;

import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.rtps.DataConsumerParticipant;
import us.ihmc.robotDataLogger.rtps.LogProducerDisplay;
import us.ihmc.robotDataLogger.rtps.VariableChangedProducer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableClient
{
   private final String serverName;

   //DDS
   private final Announcement announcement;
   private final DataConsumerParticipant dataConsumerParticipant;
   private final VariableChangedProducer variableChangedProducer;

   // Callback
   private final YoVariablesUpdatedListener yoVariablesUpdatedListener;

   // Internal values
   private final IDLYoVariableHandshakeParser handshakeParser;
   private final YoVariableRegistry debugRegistry = new YoVariableRegistry("loggerStatus");

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
      this(createParticipant(), null, listener, filters);
   }

   /**
    * Connect to an already selected log session
    * @param request
    * @param yoVariablesUpdatedListener
    */
   public YoVariableClient(DataConsumerParticipant participant, Announcement request, final YoVariablesUpdatedListener yoVariablesUpdatedListener)
   {
      this(participant, request, yoVariablesUpdatedListener, null);
   }

   private YoVariableClient(DataConsumerParticipant participant, Announcement request, final YoVariablesUpdatedListener yoVariablesUpdatedListener, LogProducerDisplay.LogSessionFilter[] filters)
   {
      this.dataConsumerParticipant = participant;
      if (request == null)
      {
         request = LogProducerDisplay.getAnnounceRequest(dataConsumerParticipant, filters);
      }

      this.serverName = request.getNameAsString();
      this.announcement = request;
      this.yoVariablesUpdatedListener = yoVariablesUpdatedListener;
      if (yoVariablesUpdatedListener.changesVariables())
      {
         this.variableChangedProducer = new VariableChangedProducer(dataConsumerParticipant);
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
         if (variableChangedProducer != null)
         {
            variableChangedProducer.disconnect();
         }

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
      if (yoVariablesUpdatedListener.changesVariables())
      {
         List<YoVariable<?>> variablesList = handshakeParser.getYoVariablesList();
         variableChangedProducer.startVariableChangedProducers(announcement, variablesList);
      }

      dataConsumerParticipant.createClearLogPubSub(announcement, yoVariablesUpdatedListener);
      dataConsumerParticipant.createTimestampListener(announcement, yoVariablesUpdatedListener);

      dataConsumerParticipant.createDataConsumer(announcement, handshakeParser, this, debugRegistry);

      state = ClientState.RUNNING;
   }

   public void requestStop()
   {
      dataConsumerParticipant.remove();
   }

   public void setSendingVariableChanges(boolean sendVariableChanges)
   {
      variableChangedProducer.setSendingChangesEnabled(sendVariableChanges);
   }

   public synchronized boolean isRunning()
   {
      return state == ClientState.RUNNING;
   }

   public void sendClearLogRequest()
   {
      try
      {
         dataConsumerParticipant.sendClearLogRequest(announcement);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public YoVariableRegistry getDebugRegistry()
   {
      return debugRegistry;
   }

   public void receivedTimestampAndData(long timestamp)
   {
      yoVariablesUpdatedListener.receivedTimestampAndData(timestamp);
   }

}
