package us.ihmc.communication.streamingData;

import java.util.concurrent.LinkedBlockingQueue;

import us.ihmc.communication.packetCommunicator.PacketProducer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.utilities.AsyncContinuousExecutor;

/**
 * User: Matt
 * Date: 1/10/13
 */
public class QueueBasedStreamingDataProducer<T extends Packet> extends PacketProducer<T>
{
   private final LinkedBlockingQueue<T> queuedData = new LinkedBlockingQueue<T>();
   private final String name;
   
   public QueueBasedStreamingDataProducer(String name)
   {
      this.name = name;
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
      AsyncContinuousExecutor.executeContinuously(runnable, name + "Queued Streaming Data Producer");
   }
}
