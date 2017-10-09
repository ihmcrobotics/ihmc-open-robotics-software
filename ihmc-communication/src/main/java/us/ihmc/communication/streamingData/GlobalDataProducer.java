package us.ihmc.communication.streamingData;

import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket.CrashLocation;
import us.ihmc.communication.packets.InvalidPacketNotificationPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.streamingData.AtomicLastPacketHolder.LastPacket;
import us.ihmc.tools.thread.ThreadTools;

public class GlobalDataProducer
{
   protected final PacketCommunicator communicator;
   private final ConcurrentLinkedQueue<Packet<?>> queuedData = new ConcurrentLinkedQueue<Packet<?>>();
   private final ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("HumanoidGlobalDataProducer"));
   private final AtomicLastPacketHolder lastPacketHolder = new AtomicLastPacketHolder();
   private final ArrayList<Class<? extends Packet<?>>> queueSkippingPackets = new ArrayList<>();

   public GlobalDataProducer(PacketCommunicator communicator)
   {
      this.communicator = communicator;
      executor.scheduleAtFixedRate(new DataProducerImpl(), 0, 1, TimeUnit.MILLISECONDS);
   }

   @SuppressWarnings("unchecked")
   public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      communicator.attachListener(clazz, listener);
      communicator.attachListener(clazz, lastPacketHolder);
   }

   public void queueDataToSend(Packet<?> packet)
   {
      queuedData.offer(packet);
   }

   public void notifyInvalidPacketReceived(Class<? extends Packet<?>> packetClass, String error)
   {
      queueDataToSend(new InvalidPacketNotificationPacket(packetClass, error));
   }

   public void notifyControllerCrash(CrashLocation location, String stackTrace)
   {
      queueDataToSend(new ControllerCrashNotificationPacket(location, stackTrace));
   }

   public void stop()
   {
      executor.shutdown();
   }

   public LastPacket getLastPacket()
   {
      return lastPacketHolder.getLastPacket();
   }

   public void setRobotTime(long time)
   {
      lastPacketHolder.setRobotTime(time);
   }

   public void skipQueueAndSend(Packet<?> packet) throws IOException
   {
      boolean successfullySentPacket = false;
      for (int i = 0; i < queueSkippingPackets.size(); i++)
      {
         if(queueSkippingPackets.get(i).isInstance(packet))
         {
            communicator.send(packet);
            successfullySentPacket = true;
            break;
         }
      }

      if(!successfullySentPacket)
         throw new IOException("The packet type " + packet.getClass().getSimpleName() +  " is not registered to skip the queue.");
   }

   /**
    * Do NOT register classes to skip the packet queue without first consulting <a href="mailto:jsmith@ihmc.us">Jesper Smith</a>.
    * @param clazz Packet class given permission to skip the packet queue and send immediately.
    */
   public void registerPacketToSkipQueue(Class<? extends Packet<?>> clazz)
   {
      queueSkippingPackets.add(clazz);
   }

   private class DataProducerImpl implements Runnable
   {

      @Override
      public void run()
      {
         Packet<?> dataObject;
         while ((dataObject = queuedData.poll()) != null)
         {
            communicator.send(dataObject);
         }
      }
   }
}
