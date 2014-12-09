package us.ihmc.robotDataCommunication;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.CRC32;

import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.SegmentedDatagramClient;
import us.ihmc.multicastLogDataProtocol.SegmentedPacketBuffer;
import us.ihmc.multicastLogDataProtocol.ThreadedLogPacketHandler;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient.LogControlVariableChangeListener;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.utilities.compression.SnappyUtils;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class YoVariableConsumer implements LogPacketHandler
{

   private final long sessionId;
   private final NetworkInterface iface;
   private final InetAddress group;

   private final List<YoVariable<?>> variables;
   private final List<JointState<?>> jointStates;
   private final YoVariablesUpdatedListener listener;

   private long packetNumber = 0;

   private int bufferSize;
   private ByteBuffer decompressed;

   private CRC32 crc32 = new CRC32();
   private SegmentedDatagramClient client;
   private ThreadedLogPacketHandler updateHandler;

   public YoVariableConsumer(long sessionId, NetworkInterface iface, byte[] group, List<YoVariable<?>> variables, List<JointState<?>> jointStates,
         YoVariablesUpdatedListener listener)
   {
      this.sessionId = sessionId;
      this.iface = iface;
      try
      {
         this.group = InetAddress.getByAddress(group);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }

      this.variables = variables;
      this.jointStates = jointStates;
      this.listener = listener;

   }

   public void start(int numberOfVariables, int numberOfJointStateVariables)
   {
      bufferSize = (1 + numberOfVariables + numberOfJointStateVariables) * 8;
      decompressed = ByteBuffer.allocate(bufferSize);

      updateHandler = new ThreadedLogPacketHandler(this, 128);
      updateHandler.start();

      client = new SegmentedDatagramClient(sessionId, iface, group, updateHandler);
      client.start();
      
   }

   private long previous;
   
   @Override
   public void newDataAvailable(SegmentedPacketBuffer buffer)
   {

      if(buffer.getUid() > previous + 1)
      {
         System.err.println("Skipped " + (buffer.getUid() - (previous + 1)) +" packets");
      }
      else if(buffer.getUid() <= previous)
      {
         System.err.println("Packet skew detected");
      }
      previous = buffer.getUid();
      
      if (packetNumber++ % listener.getDisplayOneInNPackets() == 0)
      {
         ByteBuffer buf = buffer.getBuffer();
         decompressed.clear();
         buf.clear();

         long checksum = buf.getInt() & 0xffffffffL;
         crc32.reset();
         crc32.update(buf.array(), buf.position() + buf.arrayOffset(), buf.remaining());

         if (crc32.getValue() != checksum)
         {
            System.err.println("[" + getClass().getSimpleName() + "] Checksum validation failure. Ignoring packet " + packetNumber + ".");
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

   }

   public void close()
   {
      client.close();
      updateHandler.shutdown();
   }

   @Override
   public void timestampReceived(long timestamp)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void timeout(long timeoutInMillis)
   {
      listener.receiveTimedOut(timeoutInMillis);
   }
}