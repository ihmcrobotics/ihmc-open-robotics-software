package us.ihmc.communication.streamingData;

import java.util.concurrent.LinkedBlockingQueue;

import us.ihmc.communication.packetCommunicator.PacketProducer;
import us.ihmc.communication.packets.Packet;

/**
 * User: Matt
 * Date: 1/10/13
 */
public class QueueBasedStreamingDataProducer<T extends Packet<?>> extends PacketProducer<T> implements Runnable
{
   private final LinkedBlockingQueue<T> queuedData = new LinkedBlockingQueue<T>();

   private final Thread thread;
   private volatile boolean running;

   public QueueBasedStreamingDataProducer(String name)
   {
      thread = new Thread(this, name);
   }

   public void notifyConsumers(T dataObject)
   {
      sendObject(dataObject);
   }

   public void queueDataToSend(T dataObject)
   {
      queuedData.add(dataObject);
   }

   public void run()
   {
      while(running)
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
   }

   public void startProducingData()
   {
      running = true;
      thread.start();
   }
   
   public void stopProducingData()
   {
      running = false;
      thread.interrupt();
   }
}
