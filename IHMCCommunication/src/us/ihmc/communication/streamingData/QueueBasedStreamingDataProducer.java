package us.ihmc.communication.streamingData;

import java.util.concurrent.LinkedBlockingQueue;

import us.ihmc.communication.net.ObjectProducer;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.ThreadTools;

/**
 * User: Matt
 * Date: 1/10/13
 */
public class QueueBasedStreamingDataProducer<T> extends ObjectProducer<T>
{
   private final LinkedBlockingQueue<T> queuedData = new LinkedBlockingQueue<T>();

   public QueueBasedStreamingDataProducer()
   {
   }


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
               T dataObject = null;

               try
               {
                  dataObject = queuedData.take();
               }
               catch (InterruptedException e)
               {
                  // do nothing
               }

               //               System.out.println("Notifying consumers " + data.getValue());
               if (dataObject != null)
               {
                  notifyConsumers(dataObject);
               }
         }
      };
      AsyncContinuousExecutor.executeContinuously(runnable, "Queued Streaming Data Producer");
   }
}
