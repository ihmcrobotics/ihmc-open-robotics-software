package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingRequestedEvent;

public class QuadrupedSteppingEventPacket extends Packet<QuadrupedSteppingEventPacket>
{
   private QuadrupedSteppingRequestedEvent event;

   public QuadrupedSteppingEventPacket()
   {
      this.event = null;
   }

   public QuadrupedSteppingEventPacket(QuadrupedSteppingRequestedEvent event)
   {
      this.event = event;
   }

   public QuadrupedSteppingRequestedEvent get()
   {
      return event;
   }

   @Override public boolean epsilonEquals(QuadrupedSteppingEventPacket other, double epsilon)
   {
      return (this.event == other.event);
   }
}
