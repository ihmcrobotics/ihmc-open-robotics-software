package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.PriorityBlockingQueue;

import us.ihmc.commons.Conversions;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.dataBuffers.RegistryReceiveBuffer;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoVariable;

public class RegistryConsumer extends Thread implements SubscriberListener
{
   private static final long WINDOW_SIZE = 10000; // Window size to calculate standard deviation of time between incoming packets
   private static final long DELAY = Conversions.millisecondsToNanoseconds(250);
   
   
  
//   private final ConcurrentSkipListSet<RegistryReceiveBuffer> orderedBuffers = new ConcurrentSkipListSet<>();
   private final PriorityBlockingQueue<RegistryReceiveBuffer> orderedBuffers = new PriorityBlockingQueue<>();
   private final SampleInfo sampleInfo = new SampleInfo();
   private volatile boolean running = true;
   

   private final IDLYoVariableHandshakeParser parser;
   private final List<YoVariable<?>> variables;
   
   private final ByteBuffer decompressBuffer;
   private final YoVariablesUpdatedListener listener;
   
   
   // Standard deviation calculation
   private long currentWindow = 1;
   private long previousTimestamp = - 1;
   private double average;
   private double variance;
   
   public RegistryConsumer(IDLYoVariableHandshakeParser parser, YoVariablesUpdatedListener listener)
   {
      this.parser = parser;
      this.variables = parser.getYoVariablesList();
      this.decompressBuffer = ByteBuffer.allocate(variables.size() * 8);
      this.listener = listener;
      start();
   }
   
   public void run()
   {
      while(running)
      {
         ThreadTools.sleep(1);
         
         long currentTime = System.nanoTime();

         while(orderedBuffers.size() > 250)
         {
            try
            {
               handlePackets();
            }
            catch (InterruptedException e)
            {
               // Try next time
            }
         }
      }
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
   
   private void decompressBuffer(int registryID, ByteBuffer data)
   {
      decompressBuffer.clear();
      try
      {
         SnappyUtils.uncompress(data, decompressBuffer);
      }
      catch (IllegalArgumentException | IOException e)
      {
         e.printStackTrace();
         return;
      }
      
      decompressBuffer.flip();
      LongBuffer longData = decompressBuffer.asLongBuffer();
      int numberOfVariables = longData.remaining();
      
      int offset = parser.getVariableOffset(registryID);
      for(int i = 0; i < numberOfVariables; i++)
      {
         setAndNotify(variables.get(i + offset), longData.get());
      }
      
   }
   

   private void handlePackets() throws InterruptedException
   {
      RegistryReceiveBuffer buffer = orderedBuffers.take();
      long timestamp = buffer.getTimestamp();
      
      decompressBuffer(buffer.getRegistryID(), buffer.getData());
      
      while(!orderedBuffers.isEmpty() && orderedBuffers.peek().getTimestamp() == timestamp)
      {
         RegistryReceiveBuffer next = orderedBuffers.take();
         decompressBuffer(next.getRegistryID(), next.getData());
         
      }
      listener.receivedTimestampAndData(timestamp, decompressBuffer);
      
   }

   public void close()
   {
      running = false;
      try
      {
         join();
      }
      catch (InterruptedException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
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
            if(previousTimestamp != -1)
            {
               double oldAverage = average;
               double oldTime = previousTimestamp;
               double newTime = buffer.getReceivedTimestamp();
               average = oldAverage + (newTime - oldTime)/currentWindow;
               variance = (newTime - oldTime) * (newTime - average + oldTime - oldAverage)/(currentWindow - 1);
               System.out.println(Math.sqrt(variance));
               
               if(currentWindow < WINDOW_SIZE)
               {
                  currentWindow++;
               }
            }
            previousTimestamp = buffer.getReceivedTimestamp();
            
            orderedBuffers.add(buffer);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      
   }
}
