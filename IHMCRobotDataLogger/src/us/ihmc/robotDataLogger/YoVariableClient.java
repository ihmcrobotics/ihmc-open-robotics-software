package us.ihmc.robotDataLogger;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.CRC32;

import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.StreamingDataTCPClient;
import us.ihmc.multicastLogDataProtocol.ThreadedLogPacketHandler;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient.LogControlVariableChangeListener;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.rtps.DataConsumerParticipant;
import us.ihmc.robotDataLogger.rtps.LogProducerDisplay;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.compression.SnappyUtils;

public class YoVariableClient implements LogPacketHandler
{
   private static final int RECEIVE_BUFFER_SIZE = 1024;
   private static final int TIMEOUT = 1000;

   private final CRC32 crc32 = new CRC32();
   private final String serverName;
   private final Announcement announcement;
   private final StreamingDataTCPClient streamingDataTCPClient;
   private final ThreadedLogPacketHandler threadedLogPacketHandler;
   private final YoVariablesUpdatedListener yoVariablesUpdatedListener;
   private final DataConsumerParticipant dataConsumerParticipant;
   private final IDLYoVariableHandshakeParser handshakeParser;
   private final List<YoVariable<?>> yoVariablesList;
   private final List<JointState> jointStates;
   private final int displayOneInNPackets;
   
   private ByteBuffer decompressed;
   private long previous;
   private ClientState state = ClientState.WAITING;
   private TimestampListener timestampListener;
   
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
         LogProducerDisplay.getAnnounceRequest(dataConsumerParticipant, filters);
      }
      
      
      this.serverName = request.getNameAsString();
      this.announcement = request;
      this.yoVariablesUpdatedListener = yoVariablesUpdatedListener;
      displayOneInNPackets = this.yoVariablesUpdatedListener.getDisplayOneInNPackets();

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
      
//      logControlClient = new LogControlClient(request.getControlIP(), request.getControlPort(), this.yoVariablesUpdatedListener);
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
                     if (!(changedListener instanceof LogControlVariableChangeListener))
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
      timestampListener.interrupt();
      yoVariablesUpdatedListener.receiveTimedOut();
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
      logHandshake.handshake = handshake;
      if(announcement.getModelFileDescription().getHasModel())
      {
         logHandshake.model = dataConsumerParticipant.getModelFile(announcement, timeout);
         logHandshake.modelLoaderClass = announcement.getModelFileDescription().getModelLoaderClassAsString();
         logHandshake.resourceDirectories = announcement.getModelFileDescription().getResourceDirectories().toStringArray();
         if(announcement.getModelFileDescription().getHasResourceZip())
         {
            logHandshake.resourceZip = dataConsumerParticipant.getResourceZip(announcement, timeout);
         }
         
      }
            
      yoVariablesUpdatedListener.start(logHandshake, handshakeParser);
      if (yoVariablesUpdatedListener.changesVariables())
      {
         List<YoVariable<?>> variablesList = handshakeParser.getYoVariablesList();
         dataConsumerParticipant.createVariableChangeProducer(announcement);
         logControlClient.startVariableChangedProducers(variablesList, yoVariablesUpdatedListener.executeVariableChangedListeners());
      }

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
      logControlClient.setSendingChangesEnabled(sendVariableChanges);
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
      dataConsumerParticipant.sendClearLogRequest();
   }

   @Override
   public void connected(InetSocketAddress localAddress)
   {
      System.out.println("Listening on " + localAddress);
      timestampListener = new TimestampListener(localAddress);
      timestampListener.start();
   }
   
   
   private class TimestampListener extends Thread
   {
      InetSocketAddress address;

      public TimestampListener(InetSocketAddress localAddress)
      {
         super("TimestampListener");
         
         address = new InetSocketAddress(localAddress.getAddress(), localAddress.getPort());
         
      }
      
      @Override
      public void run()
      {
         try
         {
            DatagramChannel channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true).bind(address);
            channel.configureBlocking(false);
            Selector selector = Selector.open();
            SelectionKey key = channel.register(selector, SelectionKey.OP_READ);
            ByteBuffer receiveBuffer = ByteBuffer.allocateDirect(12);
            System.out.println("Starting timestamp thread on " + address);
            while(!interrupted())
            {
               if (selector.select(TIMEOUT) > 0)
               {
                  selector.selectedKeys().remove(key);
                  if (key.isReadable())
                  {
                     receiveBuffer.clear();
                     channel.receive(receiveBuffer);
                     receiveBuffer.flip();
                     
                     if(receiveBuffer.getInt() == YoVariableProducer.TIMESTAMP_HEADER)
                     {
                        yoVariablesUpdatedListener.receivedTimestampOnly(receiveBuffer.getLong());
                     }
                     else
                     {
                        System.err.println("Received invalid timestamp, dropping");
                     }
                     
                  }
               }
            }

            System.out.println("Closing timestamp client");
            channel.close();

         }
         catch (IOException e)
         {
            e.printStackTrace();
            return;
         }

      }
   }


   @Override
   public void keepAlive()
   {
   }
}
