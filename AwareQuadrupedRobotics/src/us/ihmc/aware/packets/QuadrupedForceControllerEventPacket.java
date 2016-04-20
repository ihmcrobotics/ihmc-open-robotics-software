package us.ihmc.aware.packets;

import us.ihmc.aware.controller.force.QuadrupedForceControllerEvent;
import us.ihmc.communication.packets.Packet;

public class QuadrupedForceControllerEventPacket extends Packet<QuadrupedForceControllerEventPacket>
{
   private QuadrupedForceControllerEvent event;

   public QuadrupedForceControllerEventPacket()
   {
      this.event = null;
   }

   public QuadrupedForceControllerEventPacket(QuadrupedForceControllerEvent event)
   {
      this.event = event;
   }

   public QuadrupedForceControllerEvent get()
   {
      return event;
   }

   @Override public boolean epsilonEquals(QuadrupedForceControllerEventPacket other, double epsilon)
   {
      return (this.event == other.event);
   }
}
