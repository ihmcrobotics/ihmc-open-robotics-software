package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.PriorityBlockingQueue;

import gnu.trove.map.hash.TIntLongHashMap;
import gnu.trove.map.hash.TObjectLongHashMap;
import us.ihmc.commons.Conversions;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.dataBuffers.RegistryReceiveBuffer;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.tools.compression.CompressionImplementation;
import us.ihmc.tools.compression.CompressionImplementationFactory;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class RegistryConsumer extends Thread implements SubscriberListener
{
   private final static long TIMEOUT = Conversions.secondsToNanoseconds(10);
   
   private final static int MAXIMUM_ELEMENTS = 4096;
   
  
//   private final ConcurrentSkipListSet<RegistryReceiveBuffer> orderedBuffers = new ConcurrentSkipListSet<>();
   private final PriorityBlockingQueue<RegistryReceiveBuffer> orderedBuffers = new PriorityBlockingQueue<>();
   private final SampleInfo sampleInfo = new SampleInfo();
   private volatile boolean running = true;
   

   private final IDLYoVariableHandshakeParser parser;
   private final List<YoVariable<?>> variables;
   private final List<JointState> jointStates;
   
   private final ByteBuffer decompressBuffer;
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
   
   private long previousTimestamp = -1;
   private final YoInteger skippedPackets;
   private final YoInteger skippedIncomingPackets;
   private final YoInteger nonIncreasingTimestamps;
   private final YoInteger packetsOutOfOrder;
   private final YoInteger mergedPackets;
   private final YoInteger totalPackets;
   private final YoInteger skippedPacketDueToFullBuffer;

   private final CompressionImplementation compressionImplementation;
   
   private long lastPacketReceived;
   
   public RegistryConsumer(IDLYoVariableHandshakeParser parser, YoVariableClient yoVariableClient, YoVariableRegistry loggerDebugRegistry)
   {
      this.parser = parser;
      this.variables = parser.getYoVariablesList();
      this.jointStates = parser.getJointStates();
      this.decompressBuffer = ByteBuffer.allocate(variables.size() * 8);
      this.listener = yoVariableClient;
      
      this.skippedPackets = new YoInteger("skippedPackets", loggerDebugRegistry);
      this.skippedIncomingPackets = new YoInteger("skippedIncomingPackets", loggerDebugRegistry);
      this.nonIncreasingTimestamps = new YoInteger("nonIncreasingTimestamps", loggerDebugRegistry);
      this.packetsOutOfOrder = new YoInteger("packetsOutOfOrder", loggerDebugRegistry);
      this.mergedPackets = new YoInteger("mergedPackets", loggerDebugRegistry);
      this.totalPackets = new YoInteger("totalPackets", loggerDebugRegistry);
      this.skippedPacketDueToFullBuffer = new YoInteger("skippedPacketDueToFullBuffer", loggerDebugRegistry);
      
      this.compressionImplementation = CompressionImplementationFactory.instance();

      
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
   
   private void setAndNotify(YoVariable<?> variable, long newValue)
   {
      long previousValue = variable.getValueAsLongBits();
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
   
   private void decompressBuffer(RegistryReceiveBuffer buffer)
   {
      long previousUid = lastRegistryUid.put(buffer.getRegistryID(), buffer.getUid());
      
      updateDebugVariables(buffer, previousUid);
      
      decompressBuffer.clear();
      compressionImplementation.decompress(buffer.getData(), decompressBuffer, buffer.getNumberOfVariables() * 8);      
      decompressBuffer.flip();
      LongBuffer longData = decompressBuffer.asLongBuffer();
      
      // Sanity check
      if(longData.remaining() != buffer.getNumberOfVariables())
      {
         System.err.println("Number of variables in incoming message does not match stated number of variables. Skipping packet.");
         return;
      }
      int numberOfVariables = buffer.getNumberOfVariables();
      
      int offset = parser.getVariableOffset(buffer.getRegistryID()) + buffer.getOffset();
      for(int i = 0; i < numberOfVariables; i++)
      {
         setAndNotify(variables.get(i + offset), longData.get());
      }
      
      double[] jointStateArray = buffer.getJointStates();
      if(jointStateArray.length > 0)
      {
         DoubleBuffer jointStateBuffer = DoubleBuffer.wrap(jointStateArray);
         for(int i = 0; i < jointStates.size(); i++)
         {
            jointStates.get(i).update(jointStateBuffer);
         }         
      }
      
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
      long timestamp = buffer.getTimestamp();
      
      decompressBuffer(buffer);
      
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
      
      listener.receivedTimestampAndData(timestamp, decompressBuffer);
      
      
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
