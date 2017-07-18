package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.concurrent.PriorityBlockingQueue;

import gnu.trove.map.hash.TIntLongHashMap;
import gnu.trove.map.hash.TObjectLongHashMap;
import us.ihmc.commons.Conversions;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.robotDataLogger.LogDataType;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.dataBuffers.RegistryDecompressor;
import us.ihmc.robotDataLogger.dataBuffers.RegistryReceiveBuffer;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class RegistryConsumer extends Thread implements SubscriberListener
{
   private final static long TIMEOUT = Conversions.secondsToNanoseconds(10);
   
   private final static int MAXIMUM_ELEMENTS = 4096;
   
  
//   private final ConcurrentSkipListSet<RegistryReceiveBuffer> orderedBuffers = new ConcurrentSkipListSet<>();
   private final PriorityBlockingQueue<RegistryReceiveBuffer> orderedBuffers = new PriorityBlockingQueue<>();
   private final SampleInfo sampleInfo = new SampleInfo();
   private volatile boolean running = true;
   

   private final IDLYoVariableHandshakeParser parser;
   private final RegistryDecompressor registryDecompressor;
   private final YoVariableClient listener;
   
   private final TIntLongHashMap lastRegistryUid = new TIntLongHashMap();
   private final TObjectLongHashMap<Guid> sampleIdentities = new TObjectLongHashMap<>();
   
   // Standard deviation calculation
   private long previousTransmitTime = - 1;
   private long previousReceiveTime = - 1;
   private double jitterEstimate = 0;
   private double samples = 0;
   private double averageTimeBetweenPackets = 0;
   
   private volatile int jitterBufferSamples = 1;
   private final int segmentsForAllVariables;
   
   private long previousTimestamp = -1;
   private final YoInteger skippedPackets;
   private final YoInteger skippedIncomingPackets;
   private final YoInteger nonIncreasingTimestamps;
   private final YoInteger packetsOutOfOrder;
   private final YoInteger mergedPackets;
   private final YoInteger totalPackets;
   private final YoInteger skippedPacketDueToFullBuffer;
   private final YoInteger firstSegmentsMissing;

   
   private long lastPacketReceived;
   
   public RegistryConsumer(IDLYoVariableHandshakeParser parser, YoVariableClient yoVariableClient, YoVariableRegistry loggerDebugRegistry)
   {
      this.parser = parser;
      this.registryDecompressor = new RegistryDecompressor(parser.getYoVariablesList(), parser.getJointStates());
      this.listener = yoVariableClient;
      
      this.skippedPackets = new YoInteger("skippedPackets", loggerDebugRegistry);
      this.skippedIncomingPackets = new YoInteger("skippedIncomingPackets", loggerDebugRegistry);
      this.nonIncreasingTimestamps = new YoInteger("nonIncreasingTimestamps", loggerDebugRegistry);
      this.packetsOutOfOrder = new YoInteger("packetsOutOfOrder", loggerDebugRegistry);
      this.mergedPackets = new YoInteger("mergedPackets", loggerDebugRegistry);
      this.totalPackets = new YoInteger("totalPackets", loggerDebugRegistry);
      this.skippedPacketDueToFullBuffer = new YoInteger("skippedPacketDueToFullBuffer", loggerDebugRegistry);
      this.firstSegmentsMissing = new YoInteger("firstSegmentsMissing", loggerDebugRegistry);
      

      this.segmentsForAllVariables = LogParticipantTools.calculateLogSegmentSizes(parser.getNumberOfVariables(), parser.getNumberOfJointStateVariables()).length;
      
      start();
   }
   
   public void run()
   {
      
      lastPacketReceived = System.nanoTime();
      while(running)
      {
         ThreadTools.sleep(1);
         
         while(orderedBuffers.size() > (jitterBufferSamples + lastRegistryUid.size() + segmentsForAllVariables + 1))
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
         
         if(System.nanoTime() - lastPacketReceived > TIMEOUT)
         {
            running = false;
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
            packetsOutOfOrder.increment();
         }
         else
         {
            skippedPackets.add((int) (buffer.getUid() - previousUid - 1));
         }
      }
      
     
      totalPackets.increment();
   }
   

   private void handlePackets() throws InterruptedException
   {
      RegistryReceiveBuffer buffer = orderedBuffers.take();
      if(buffer.getType() == LogDataType.DATA_PACKET)
      {
      
         long timestamp = buffer.getTimestamp();
         
         decompressBuffer(buffer);
         
         if(buffer.getOffset() > 0)
         {
            firstSegmentsMissing.increment();
         }
         
         while(!orderedBuffers.isEmpty() && orderedBuffers.peek().getTimestamp() == timestamp)
         {
            RegistryReceiveBuffer next = orderedBuffers.take();
            decompressBuffer(next);
            mergedPackets.increment();
         }
         
         if(previousTimestamp != -1 && previousTimestamp >= buffer.getTimestamp())
         {
            nonIncreasingTimestamps.increment();         
         }
         previousTimestamp = buffer.getTimestamp();
         
         listener.receivedTimestampAndData(timestamp);
      
      }
      else
      {
         //Received keep alive, ignore
      }
   }
   
   @Override
   public void onSubscriptionMatched(Subscriber subscriber, MatchingInfo info)
   {
   }
   
   @Override
   public void onNewDataMessage(Subscriber subscriber)
   {
      RegistryReceiveBuffer buffer = new RegistryReceiveBuffer(System.nanoTime());
      try
      {
         if(subscriber.takeNextData(buffer, sampleInfo))
         {
            // RFC 1889 jitter estimate
            if(previousTransmitTime != -1)
            {
               long D = (buffer.getReceivedTimestamp() - previousReceiveTime) - (buffer.getTransmitTime() - previousTransmitTime);
               if(D < 0) D = -D;
               
               jitterEstimate += (D - jitterEstimate)/16;
               
               ++samples;
               averageTimeBetweenPackets += ((buffer.getTransmitTime() - previousTransmitTime) - averageTimeBetweenPackets) / samples;
               
               jitterBufferSamples = (int) (Math.ceil(jitterEstimate / averageTimeBetweenPackets) + 1);
               
               if(jitterBufferSamples > MAXIMUM_ELEMENTS / 2)
               {
                  jitterBufferSamples = MAXIMUM_ELEMENTS / 2;
               }
            }
            previousTransmitTime = buffer.getTransmitTime();
            previousReceiveTime = buffer.getReceivedTimestamp();
            
            if(sampleIdentities.get(sampleInfo.getSampleIdentity().getGuid()) != sampleIdentities.getNoEntryValue() && sampleIdentities.get(sampleInfo.getSampleIdentity().getGuid()) + 1 != sampleInfo.getSampleIdentity().getSequenceNumber().get())
            {
               skippedIncomingPackets.increment();
            }
            sampleIdentities.put(sampleInfo.getSampleIdentity().getGuid(), sampleInfo.getSampleIdentity().getSequenceNumber().get());
            
            
            if(orderedBuffers.size() < MAXIMUM_ELEMENTS)
            {
               orderedBuffers.add(buffer);
            }
            else
            {
               skippedPacketDueToFullBuffer.increment();
            }
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      
   }
}
