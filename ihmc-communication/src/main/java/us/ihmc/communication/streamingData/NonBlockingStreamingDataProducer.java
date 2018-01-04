package us.ihmc.communication.streamingData;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.packetCommunicator.PacketProducer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.commons.thread.ThreadTools;

public class NonBlockingStreamingDataProducer<T extends Packet> extends PacketProducer<T>
{
   private final ConcurrentLinkedQueue<T> queuedData = new ConcurrentLinkedQueue<T>(); 
   private volatile boolean running = true;
   
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
            while (running)
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
   
   public void stop()
   {
      this.running = false;
   }
}
