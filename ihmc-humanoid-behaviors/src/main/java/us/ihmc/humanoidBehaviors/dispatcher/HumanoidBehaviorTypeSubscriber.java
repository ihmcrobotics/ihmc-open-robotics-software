package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.HumanoidBehaviorTypePacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.log.LogTools;

public class HumanoidBehaviorTypeSubscriber implements PacketConsumer<HumanoidBehaviorTypePacket>, BehaviorTypeSubscriber<HumanoidBehaviorType>
{
   private final AtomicReference<HumanoidBehaviorTypePacket> packetReference = new AtomicReference<HumanoidBehaviorTypePacket>(null);

   public HumanoidBehaviorTypeSubscriber()
   {
      
   }

   /* (non-Javadoc)
    * @see us.ihmc.humanoidBehaviors.dispatcher.BehaviorTypeSubscriber#checkForNewBehaviorRequested()
    */
   @Override
   public boolean checkForNewBehaviorRequested()
   {
      return packetReference.get() != null;
   }

   /* (non-Javadoc)
    * @see us.ihmc.humanoidBehaviors.dispatcher.BehaviorTypeSubscriber#getRequestedBehavior()
    */
   @Override
   public HumanoidBehaviorType getRequestedBehavior()
   {
      return HumanoidBehaviorType.fromByte(packetReference.getAndSet(null).getHumanoidBehaviorType());
   }

   @Override
   public void receivedPacket(HumanoidBehaviorTypePacket object)
   {
      packetReference.set(object);
      LogTools.info("Received behavior packet of type: " + object.getHumanoidBehaviorType());
   }
}
