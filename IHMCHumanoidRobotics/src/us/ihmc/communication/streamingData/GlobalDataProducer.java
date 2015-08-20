package us.ihmc.communication.streamingData;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket.CrashLocation;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.InvalidPacketNotificationPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.tools.thread.ThreadTools;

public class GlobalDataProducer
{
   private final PacketCommunicator communicator;
   private final ConcurrentLinkedQueue<Packet<?>> queuedData = new ConcurrentLinkedQueue<Packet<?>>();
   private final ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("GlobalDataProducer"));

   public GlobalDataProducer(PacketCommunicator communicator)
   {
      this.communicator = communicator;
      executor.scheduleAtFixedRate(new DataProducerImpl(), 0, 1, TimeUnit.MILLISECONDS);
   }

   public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      communicator.attachListener(clazz, listener);
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

   /**
    * Special method to directly send RobotConfigurationData, skipping the queue
    * @param robotConfigData
    */
   public void send(RobotConfigurationData robotConfigData)
   {
      communicator.send(robotConfigData);
   }

   /**
    * Special method to directly send HandJointAnglePacket, skipping the queue
    * @param handJointAnglePacket
    */
   public void send(HandJointAnglePacket handJointAnglePacket)
   {
      communicator.send(handJointAnglePacket);
   }
   

   /**
    * Special method to directly send CapturabilityBasedStatus, skipping the queue
    * 
    * @param status
    */
   public void send(CapturabilityBasedStatus status)
   {
      communicator.send(status);
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
