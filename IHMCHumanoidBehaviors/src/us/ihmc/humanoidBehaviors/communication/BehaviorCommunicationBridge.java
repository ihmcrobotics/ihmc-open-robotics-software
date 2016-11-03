package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class BehaviorCommunicationBridge implements OutgoingCommunicationBridgeInterface, IncomingCommunicationBridgeInterface
{
   private final PacketCommunicator behaviorPacketCommunicator;

   public BehaviorCommunicationBridge(PacketCommunicator behaviorPacketCommunicator)
   {
      this.behaviorPacketCommunicator = behaviorPacketCommunicator;
   }

   @Override
   public void sendPacketToController(Packet packet)
   {
      if (behaviorPacketCommunicator.isConnected())
      {
         packet.setDestination(PacketDestination.CONTROLLER.ordinal());
         behaviorPacketCommunicator.send(packet);
      }
   }

   @Override
   public void sendPacketToUI(Packet packet)
   {
      if (behaviorPacketCommunicator.isConnected())
      {
         packet.setDestination(PacketDestination.UI.ordinal());

         behaviorPacketCommunicator.send(packet);
      }
   }

   @Override
   public void sendPacketToBehavior(Packet packet)
   {
      if (behaviorPacketCommunicator.isConnected())
      {
         packet.setDestination(PacketDestination.BEHAVIOR_MODULE.ordinal());
         behaviorPacketCommunicator.send(packet);
      }
   }

   @Override
   public void sendPacketToNetworkProcessor(Packet obj)
   {
      behaviorPacketCommunicator.send(obj);
   }

   @Override
   public void attachGlobalListener(GlobalPacketConsumer listener)
   {
      behaviorPacketCommunicator.attachGlobalListener(listener);
   }

   @Override
   public void detachGlobalListener(GlobalPacketConsumer listener)
   {
      behaviorPacketCommunicator.detachGlobalListener(listener);
   }


   @Override
   public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      behaviorPacketCommunicator.attachListener(clazz, listener);
   }

   @Override
   public <T extends Packet> void detachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      behaviorPacketCommunicator.detachListener(clazz, listener);
   }

}
