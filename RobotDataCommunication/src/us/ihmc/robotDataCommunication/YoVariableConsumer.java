package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.CRC32;

import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.StreamingDataTCPClient;
import us.ihmc.multicastLogDataProtocol.ThreadedLogPacketHandler;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient.LogControlVariableChangeListener;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.utilities.compression.SnappyUtils;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class YoVariableConsumer implements LogPacketHandler
{
   private final int RECEIVE_BUFFER_SIZE = 1024;

   private final InetAddress dataIP;

   private final List<YoVariable<?>> variables;
   private final List<JointState<?>> jointStates;
   private final YoVariablesUpdatedListener listener;

   private ByteBuffer decompressed;

   private CRC32 crc32 = new CRC32();
   private final StreamingDataTCPClient client;
   private final ThreadedLogPacketHandler updateHandler;

   public YoVariableConsumer(byte[] dataIP, int port, List<YoVariable<?>> variables, List<JointState<?>> jointStates, YoVariablesUpdatedListener listener)
   {
      try
      {
         this.dataIP = InetAddress.getByAddress(dataIP);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }

      this.variables = variables;
      this.jointStates = jointStates;
      this.listener = listener;

      updateHandler = new ThreadedLogPacketHandler(this, RECEIVE_BUFFER_SIZE);
      client = new StreamingDataTCPClient(this.dataIP, port, updateHandler, listener.getDisplayOneInNPackets());
   }

   public void start(int bufferSize)
   {
      decompressed = ByteBuffer.allocate(bufferSize);

      updateHandler.start();

      client.start();

   }

   private long previous;

   @Override
   public void newDataAvailable(LogDataHeader header, ByteBuffer buf)
   {

      if (header.getUid() > previous + 1)
      {
         System.err.println("Skipped " + (header.getUid() - (previous + 1)) + " packets");
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

      for (int i = 0; i < variables.size(); i++)
      {
         YoVariable<?> variable = variables.get(i);
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

      listener.receivedUpdate(timestamp, decompressed);

   }

   public void requestStopS()
   {
      if (client.isRunning())
      {
         client.requestStop();
      }
   }

   @Override
   public void timestampReceived(long timestamp)
   {
      listener.timestampReceived(timestamp);
   }

   @Override
   public void timeout()
   {
      listener.receiveTimedOut();
      updateHandler.shutdown();
   }

   public LogHandshake getHandshake()
   {
      try
      {
         return client.getHandshake();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}