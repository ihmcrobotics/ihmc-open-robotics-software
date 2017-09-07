package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.communication.packets.Packet;

public class QuadrupedForceControllerEventPacket extends Packet<QuadrupedForceControllerEventPacket>
{
   private QuadrupedForceControllerRequestedEvent event;

   public QuadrupedForceControllerEventPacket()
   {
      this.event = null;
   }

   public QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent event)
   {
      this.event = event;
   }

   public QuadrupedForceControllerRequestedEvent get()
   {
      return event;
   }

   @Override public boolean epsilonEquals(QuadrupedForceControllerEventPacket other, double epsilon)
   {
      return (this.event == other.event);
   }
}
