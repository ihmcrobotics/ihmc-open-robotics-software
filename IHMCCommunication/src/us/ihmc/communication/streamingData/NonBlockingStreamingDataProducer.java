package us.ihmc.communication.streamingData;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.ObjectProducer;
import us.ihmc.utilities.ThreadTools;

public class NonBlockingStreamingDataProducer<T> extends ObjectProducer<T>
{
   private final ConcurrentLinkedQueue<T> queuedData = new ConcurrentLinkedQueue<T>(); 
   
   
   public void notifyConsumers(T dataObject)
   {
      sendObject(dataObject);
   }

   public void queueDataToSend(T dataObject)
   {
      queuedData.add(dataObject);
   }

   public void startProducingData()
   {
      Runnable runnable = new Runnable()
      {

         public void run()
         {
            while (true)
            {
               T dataObject;
               while((dataObject = queuedData.poll()) != null)
               {
                     notifyConsumers(dataObject);
               }
               
               ThreadTools.sleep(100);

            }
         }
      };
      ThreadTools.startAsDaemon(runnable, "Streaming Data Producer");
   }
}
