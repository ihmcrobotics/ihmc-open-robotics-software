package us.ihmc.communication.streamingData;

import java.util.ArrayList;

import us.ihmc.tools.thread.ThreadTools;


public class SimpleStreamingDataProducer implements StreamingDataProducer
{
   private ArrayList<StreamingDataConsumer> consumers = new ArrayList<StreamingDataConsumer>();
   private final long dataIdentifier;
   
   public SimpleStreamingDataProducer()
   {
      this(SimpleStreamedData.getSerialVersionUID());
   }
   
   public SimpleStreamingDataProducer(long dataIdentifier)
   {
      this.dataIdentifier = dataIdentifier;
   }
   
   public void registerConsumer(StreamingDataConsumer consumer)
   {
      consumers.add(consumer);
   }
   
   public void notifyConsumers(SimpleStreamedData dataObject)
   {
      for (StreamingDataConsumer consumer : consumers)
      {
         consumer.consume(getDataIdentifier(), dataObject);
      }
   }
   
   public void startProducingData()
   {
      Runnable runnable = new Runnable()
      {
         private int value = 0;
         
         public void run()
         {
            while(true)
            {
               SimpleStreamedData data = new SimpleStreamedData(value++);
//               System.out.println("Notifying consumers " + data.getValue());
               notifyConsumers(data);
               ThreadTools.sleep(10L);
            } 
         } 
      };
      
      Thread thread = new Thread(runnable);
      thread.setDaemon(true);
      thread.start();
   }

   public long getDataIdentifier()
   {
      return dataIdentifier;
   }
}
