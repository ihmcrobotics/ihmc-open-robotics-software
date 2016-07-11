package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.CRC32;

import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.StreamingDataTCPClient;
import us.ihmc.multicastLogDataProtocol.ThreadedLogPacketHandler;
import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionDisplay;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient.LogControlVariableChangeListener;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.compression.SnappyUtils;

public class YoVariableClient implements LogPacketHandler
{
   private static final int RECEIVE_BUFFER_SIZE = 1024;

   private final CRC32 crc32 = new CRC32();
   private final StreamingDataTCPClient streamingDataTCPClient;
   private final ThreadedLogPacketHandler threadedLogPacketHandler;
   private final YoVariablesUpdatedListener yoVariablesUpdatedListener;
   private final LogControlClient logControlClient;
   private final YoVariableHandshakeParser handshakeParser;
   private final List<YoVariable<?>> yoVariablesList;
   private final List<JointState<?>> jointStates;
   private final int displayOneInNPackets;
   
   private ByteBuffer decompressed;
   private long previous;
   private ClientState state = ClientState.WAITING;

   private enum ClientState
   {
      WAITING, RUNNING, STOPPED
   }

   public YoVariableClient(YoVariablesUpdatedListener listener, String registryPrefix)
   {
      this(LogSessionDisplay.getAnnounceRequest(), listener, registryPrefix);
   }

   public YoVariableClient(YoVariablesUpdatedListener listener, String registryPrefix, LogSessionDisplay.RobotIPToNameRemapHandler remapHandler)
   {
      this(LogSessionDisplay.getAnnounceRequest(remapHandler), listener, registryPrefix);
   }

   public YoVariableClient(AnnounceRequest request, final YoVariablesUpdatedListener yoVariablesUpdatedListener, String registryPrefix)
   {      
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
      
      logControlClient = new LogControlClient(request.getControlIP(), request.getControlPort(), this.yoVariablesUpdatedListener);
      handshakeParser = new YoVariableHandshakeParser(registryPrefix, this.yoVariablesUpdatedListener.populateRegistry());
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

      yoVariablesUpdatedListener.receivedTimestampAndData(timestamp, decompressed);
   }

   @Override
   public void timestampReceived(long timestamp)
   {
      yoVariablesUpdatedListener.receivedTimestampOnly(timestamp);
   }

   @Override
   public void timeout()
   {
      threadedLogPacketHandler.shutdown();
      yoVariablesUpdatedListener.receiveTimedOut();
   }
   
   public LogHandshake getHandshake()
   {
      try
      {
         return streamingDataTCPClient.getHandshake();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void start()
   {
      try
      {
         start(-1L);
      }
      catch (SocketTimeoutException e)
      {
         throw new RuntimeException(e);
      }
   }

   public synchronized void start(long timeout) throws SocketTimeoutException
   {
      if (state != ClientState.WAITING)
      {
         throw new RuntimeException("Client already started");
      }

      System.out.println("Requesting handshake");
      LogHandshake handshake = getHandshake();

      logControlClient.connect();
      handshakeParser.parseFrom(handshake.protoShake);

      yoVariablesUpdatedListener.start(handshake, handshakeParser);
      if (yoVariablesUpdatedListener.changesVariables())
      {
         List<YoVariable<?>> variablesList = handshakeParser.getYoVariablesList();
         logControlClient.startVariableChangedProducers(variablesList, yoVariablesUpdatedListener.executeVariableChangedListeners());
      }

      decompressed = ByteBuffer.allocate(handshakeParser.getBufferSize());

      threadedLogPacketHandler.start();
      streamingDataTCPClient.start();

      state = ClientState.RUNNING;
   }

   public void requestStop()
   {
      System.out.println(streamingDataTCPClient.isRunning());
      if (streamingDataTCPClient.isRunning())
      {
         System.out.println("HELP");
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
         logControlClient.close();
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
      logControlClient.sendClearLogRequest();
   }
}
