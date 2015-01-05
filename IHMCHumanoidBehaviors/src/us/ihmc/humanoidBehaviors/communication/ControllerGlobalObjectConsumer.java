package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;

public class ControllerGlobalObjectConsumer implements PacketConsumer
{
   private final BehaviorInterface behavior;
   public ControllerGlobalObjectConsumer(BehaviorInterface behavior)
   {
      this.behavior = behavior;
   }

   @Override
   public void receivedPacket(Packet packet)
   {
      behavior.consumeObjectFromController(packet);
   }
}
