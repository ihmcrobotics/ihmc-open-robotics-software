package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.PassPacketBehavior;

public class PassMessageTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private Packet packet;
   private PassPacketBehavior passMessageBehavior;

   public PassMessageTask(Packet packet, PassPacketBehavior passMessageBehavior)
   {
      this(null, packet, passMessageBehavior);
   }

   public PassMessageTask(E stateEnum, Packet packet, PassPacketBehavior passMessageBehavior)
   {
      super(stateEnum, passMessageBehavior);
      this.passMessageBehavior = passMessageBehavior;
      this.packet = packet;
   }

   @Override
   protected void setBehaviorInput()
   {
      passMessageBehavior.setPacket(packet);
   }
}
