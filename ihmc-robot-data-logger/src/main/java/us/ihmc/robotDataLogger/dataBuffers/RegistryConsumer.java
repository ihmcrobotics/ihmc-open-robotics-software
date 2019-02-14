package us.ihmc.robotDataLogger.dataBuffers;

import java.util.concurrent.PriorityBlockingQueue;

import gnu.trove.map.hash.TIntLongHashMap;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotDataLogger.LogDataType;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.util.DebugRegistry;

public class RegistryConsumer extends Thread
{
   private final static int MAXIMUM_ELEMENTS = 4096;
   
  
//   private final ConcurrentSkipListSet<RegistryReceiveBuffer> orderedBuffers = new ConcurrentSkipListSet<>();
   private final PriorityBlockingQueue<RegistryReceiveBuffer> orderedBuffers = new PriorityBlockingQueue<>();
   private volatile boolean running = true;
   
   private boolean firstSample = true;

   private final IDLYoVariableHandshakeParser parser;
   private final RegistryDecompressor registryDecompressor;
   private final YoVariableClientImplementation listener;
   
   private final TIntLongHashMap lastRegistryUid = new TIntLongHashMap();
   
   // Standard deviation calculation
   private long previousTransmitTime = - 1;
   private long previousReceiveTime = - 1;
   private double jitterEstimate = 0;
   private double samples = 0;
   private double averageTimeBetweenPackets = 0;
   
   private volatile int jitterBufferSamples = 1;
   
   private long previousTimestamp = -1;

   
   private long lastPacketReceived;
   
   private final DebugRegistry debugRegistry;
   
   public RegistryConsumer(IDLYoVariableHandshakeParser parser, YoVariableClientImplementation yoVariableClient, DebugRegistry debugRegistry)
   {
      this.parser = parser;
      this.registryDecompressor = new RegistryDecompressor(parser.getYoVariablesList(), parser.getJointStates());
      this.listener = yoVariableClient;

      this.debugRegistry = debugRegistry;

      
      start();
   }
   
   public void run()
   {
      
      lastPacketReceived = System.nanoTime();
      while(running)
      {
         ThreadTools.sleep(1);
         
         while(orderedBuffers.size() > (jitterBufferSamples + lastRegistryUid.size() + 1))
         {
            try
            {
               handlePackets();
               lastPacketReceived = System.nanoTime();
            }
            catch (InterruptedException e)
            {
               // Try next time
            }
         }
      }
      
      // Empty buffer
      while(!orderedBuffers.isEmpty())
      {
         try
         {
            handlePackets();
         }
         catch (InterruptedException e)
         {
         }
      }
      
      listener.connectionClosed();
      
   }
   
   public void stopImmediatly()
   {
      running = false;
   }
   

   
   private void decompressBuffer(RegistryReceiveBuffer buffer)
   {
      long previousUid = lastRegistryUid.put(buffer.getRegistryID(), buffer.getUid());
      
      updateDebugVariables(buffer, previousUid);
      
      registryDecompressor.decompressSegment(buffer, parser.getVariableOffset(buffer.getRegistryID()));
      
      
   }

   void updateDebugVariables(RegistryReceiveBuffer buffer, long previousUid)
   {
      if(previousUid != lastRegistryUid.getNoEntryValue() && buffer.getUid() != previousUid + 1)
      {
         if(buffer.getUid() < previousUid)
         {
            debugRegistry.getPacketsOutOfOrder().increment();
         }
         else
         {
            debugRegistry.getSkippedPackets().add((int) (buffer.getUid() - previousUid - 1));
         }
      }
      
     
      debugRegistry.getTotalPackets().increment();
   }
   

   private void handlePackets() throws InterruptedException
   {
      RegistryReceiveBuffer buffer = orderedBuffers.take();
      if(buffer.getType() == LogDataType.DATA_PACKET)
      {
      
         long timestamp = buffer.getTimestamp();
         
         decompressBuffer(buffer);
         
         while(!orderedBuffers.isEmpty() && orderedBuffers.peek().getTimestamp() == timestamp)
         {
            RegistryReceiveBuffer next = orderedBuffers.take();
            decompressBuffer(next);
            debugRegistry.getMergedPackets().increment();
         }
         
         if(previousTimestamp != -1 && previousTimestamp >= buffer.getTimestamp())
         {
            debugRegistry.getNonIncreasingTimestamps().increment();         
         }
         previousTimestamp = buffer.getTimestamp();
         
         if(firstSample)
         {
            listener.connected();
            firstSample = false;
         }
         else
         {
            listener.receivedTimestampAndData(timestamp);
         }
      }
      else
      {
         //Received keep alive, ignore
      }
   }
   
   public void onNewDataMessage(RegistryReceiveBuffer buffer)
   {
      // RFC 1889 jitter estimate
      if (previousTransmitTime != -1)
      {
         long D = (buffer.getReceivedTimestamp() - previousReceiveTime) - (buffer.getTransmitTime() - previousTransmitTime);
         if (D < 0)
            D = -D;

         jitterEstimate += (D - jitterEstimate) / 16;

         ++samples;
         averageTimeBetweenPackets += ((buffer.getTransmitTime() - previousTransmitTime) - averageTimeBetweenPackets) / samples;

         jitterBufferSamples = (int) (Math.ceil(jitterEstimate / averageTimeBetweenPackets) + 1);

         if (jitterBufferSamples > MAXIMUM_ELEMENTS / 2)
         {
            jitterBufferSamples = MAXIMUM_ELEMENTS / 2;
         }
      }
      previousTransmitTime = buffer.getTransmitTime();
      previousReceiveTime = buffer.getReceivedTimestamp();

      

      if (orderedBuffers.size() < MAXIMUM_ELEMENTS)
      {
         orderedBuffers.add(buffer);
      }
      else
      {
         debugRegistry.getSkippedPacketDueToFullBuffer().increment();
      }
   }
}
