package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.PassPacketBehavior;

public class PassMessageTask extends BehaviorAction
{
   private Packet<?> packet;
   private PassPacketBehavior passMessageBehavior;

   public PassMessageTask(Packet<?> packet, PassPacketBehavior passMessageBehavior)
   {
      super(passMessageBehavior);
      this.passMessageBehavior = passMessageBehavior;
      this.packet = packet;
   }

   @Override
   protected void setBehaviorInput()
   {
      passMessageBehavior.setPacket(packet);
   }
}
