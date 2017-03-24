package us.ihmc.robotDataLogger;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.CRC32;

import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.StreamingDataTCPClient;
import us.ihmc.multicastLogDataProtocol.ThreadedLogPacketHandler;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.rtps.DataConsumerParticipant;
import us.ihmc.robotDataLogger.rtps.LogProducerDisplay;
import us.ihmc.robotDataLogger.rtps.VariableChangedProducer;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.compression.SnappyUtils;

public class YoVariableClient implements LogPacketHandler
{
   private static final int RECEIVE_BUFFER_SIZE = 1024;

   private final CRC32 crc32 = new CRC32();
   private final String serverName;
   
   //DDS
   private final Announcement announcement;
   private final DataConsumerParticipant dataConsumerParticipant;
   private final VariableChangedProducer variableChangedProducer;

   // Streaming protocol
   private final StreamingDataTCPClient streamingDataTCPClient;
   private final ThreadedLogPacketHandler threadedLogPacketHandler;
   
   // Callback
   private final YoVariablesUpdatedListener yoVariablesUpdatedListener;
   
   // Internal values
   private final IDLYoVariableHandshakeParser handshakeParser;
   private final List<YoVariable<?>> yoVariablesList;
   private final List<JointState> jointStates;
   private final int displayOneInNPackets;
   
   private ByteBuffer decompressed;
   private long previous;
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
    * @param registryPrefix
    * @param filters
    */
   public YoVariableClient(YoVariablesUpdatedListener listener, String registryPrefix, LogProducerDisplay.LogSessionFilter... filters)
   {
      this(createParticipant(), null, listener, registryPrefix, filters);
   }

   /**
    * Connect to an already selected log session
    * 
    * @param request
    * @param yoVariablesUpdatedListener
    * @param registryPrefix
    */
   public YoVariableClient(DataConsumerParticipant participant, Announcement request, final YoVariablesUpdatedListener yoVariablesUpdatedListener, String registryPrefix)
   {
      this(participant, request, yoVariablesUpdatedListener, registryPrefix, null);
   }
   
   
   private YoVariableClient(DataConsumerParticipant participant, Announcement request, final YoVariablesUpdatedListener yoVariablesUpdatedListener, String registryPrefix, LogProducerDisplay.LogSessionFilter[] filters)
   {   
      this.dataConsumerParticipant = participant;
      if(request == null)
      {
         request = LogProducerDisplay.getAnnounceRequest(dataConsumerParticipant, filters);
      }
      
      
      this.serverName = request.getNameAsString();
      this.announcement = request;
      this.yoVariablesUpdatedListener = yoVariablesUpdatedListener;
      this.displayOneInNPackets = this.yoVariablesUpdatedListener.getDisplayOneInNPackets();
      if(yoVariablesUpdatedListener.changesVariables())
      {
         this.variableChangedProducer = new VariableChangedProducer(dataConsumerParticipant);
      }
      else
      {
         this.variableChangedProducer = null;
      }

      InetAddress inetAddress;
      try
      {
         inetAddress = InetAddress.getByAddress(request.getDataIP());
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }

      threadedLogPacketHandler = new ThreadedLogPacketHandler(this, RECEIVE_BUFFER_SIZE);
      streamingDataTCPClient = new StreamingDataTCPClient(inetAddress, request.getDataPort(), threadedLogPacketHandler, displayOneInNPackets);
      
      handshakeParser = new IDLYoVariableHandshakeParser(HandshakeFileType.IDL_CDR, registryPrefix);
      yoVariablesList = handshakeParser.getYoVariablesList();
      jointStates = handshakeParser.getJointStates();

      this.yoVariablesUpdatedListener.setYoVariableClient(this);
   }

   @Override
   public void newDataAvailable(LogDataHeader header, ByteBuffer buf)
   {
      if (header.getUid() > previous + displayOneInNPackets)
      {
         System.err.println("Skipped " + (header.getUid() - previous - displayOneInNPackets) + " packets");
      }
      else if (header.getUid() <= previous)
      {
         System.err.println("Packet skew detected " + header.getUid());
      }

      previous = header.getUid();
      decompressed.clear();
      buf.clear();

      long checksum = header.getCrc32() & 0xFFFFFFFFL;
      crc32.reset();
      crc32.update(buf.array(), buf.position() + buf.arrayOffset(), buf.remaining());

      if (crc32.getValue() != checksum)
      {
         System.err.println("[" + getClass().getSimpleName() + "] Checksum validation failure. Ignoring packet " + header.getUid() + ".");
         return;
      }

      try
      {
         SnappyUtils.uncompress(buf, decompressed);
         decompressed.flip();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return;
      }

      long timestamp = decompressed.getLong();

      if(this.yoVariablesUpdatedListener.updateYoVariables())
      {
         LongBuffer data = decompressed.asLongBuffer();
         for (int i = 0; i < yoVariablesList.size(); i++)
         {
            YoVariable<?> variable = yoVariablesList.get(i);
            long previousValue = variable.getValueAsLongBits();
            long newValue = data.get();
            variable.setValueFromLongBits(newValue, false);
            if (previousValue != newValue)
            {
               ArrayList<VariableChangedListener> changedListeners = variable.getVariableChangedListeners();
               if (changedListeners != null)
               {
                  for (int listener = 0; listener < changedListeners.size(); listener++)
                  {
                     VariableChangedListener changedListener = changedListeners.get(listener);
                     if (!(changedListener instanceof VariableChangedProducer.VariableListener))
                     {
                        changedListener.variableChanged(variable);
                     }
                  }
               }
            }
         }
         
         for (int i = 0; i < jointStates.size(); i++)
         {
            jointStates.get(i).update(data);
         }         
      }

      yoVariablesUpdatedListener.receivedTimestampAndData(timestamp, decompressed);
   }
   
   public String getServerName()
   {
      return serverName;
   }

   @Override
   public void timestampReceived(long timestamp)
   {
   }

   @Override
   public void timeout()
   {
      threadedLogPacketHandler.shutdown();
      yoVariablesUpdatedListener.receiveTimedOut();
      dataConsumerParticipant.remove();
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
      if(announcement.getModelFileDescription().getHasModel())
      {
         logHandshake.setModel(dataConsumerParticipant.getModelFile(announcement, timeout));
         logHandshake.setModelLoaderClass(announcement.getModelFileDescription().getModelLoaderClassAsString());
         logHandshake.setResourceDirectories(announcement.getModelFileDescription().getResourceDirectories().toStringArray());
         if(announcement.getModelFileDescription().getHasResourceZip())
         {
            logHandshake.setResourceZip(dataConsumerParticipant.getResourceZip(announcement, timeout));
         }
         
      }
            
      yoVariablesUpdatedListener.start(logHandshake, handshakeParser);
      if (yoVariablesUpdatedListener.changesVariables())
      {
         List<YoVariable<?>> variablesList = handshakeParser.getYoVariablesList();
         variableChangedProducer.startVariableChangedProducers(announcement, variablesList);
      }
      
      dataConsumerParticipant.createClearLogPubSub(announcement, yoVariablesUpdatedListener);
      dataConsumerParticipant.createTimestampListener(announcement, yoVariablesUpdatedListener);
      
      decompressed = ByteBuffer.allocate(handshakeParser.getBufferSize());

      threadedLogPacketHandler.start();
      streamingDataTCPClient.start();

      state = ClientState.RUNNING;
   }

   public void requestStop()
   {
      if (streamingDataTCPClient.isRunning())
      {
         streamingDataTCPClient.requestStop();
      }
   }
   
   public void setSendingVariableChanges(boolean sendVariableChanges)
   {
      variableChangedProducer.setSendingChangesEnabled(sendVariableChanges);
   }
   
   public synchronized void disconnected()
   {
      if (state == ClientState.RUNNING)
      {
         dataConsumerParticipant.remove();
         yoVariablesUpdatedListener.disconnected();

         state = ClientState.STOPPED;
      }
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

   @Override
   public void keepAlive()
   {
   }

   @Override
   public void connected(InetSocketAddress localAddress)
   {
   }

}
