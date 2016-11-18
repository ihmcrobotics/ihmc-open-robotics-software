package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class PassPacketBehavior extends AbstractBehavior
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private Packet packet;

   public PassPacketBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);

   }

   @Override
   public void doControl()
   {

      if (!packetHasBeenSent.getBooleanValue() && (packet != null))
      {
         forwardPacket();
      }

   }

   public void setPacket(Packet packet)
   {
      this.packet = packet;
   }

   private void forwardPacket()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         sendPacket(packet);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void initialize()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public boolean isDone()
   {
      return packetHasBeenSent.getBooleanValue() && !isPaused.getBooleanValue();
   }

}
