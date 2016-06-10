package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.communication.packets.Packet;

public class QuadrupedForceControllerStatePacket extends Packet<QuadrupedForceControllerStatePacket>
{
   private QuadrupedForceControllerState state;

   public QuadrupedForceControllerStatePacket()
   {
      this.state = null;
   }

   public QuadrupedForceControllerStatePacket(QuadrupedForceControllerState state)
   {
      this.state = state;
   }

   public QuadrupedForceControllerState get()
   {
      return state;
   }

   @Override public boolean epsilonEquals(QuadrupedForceControllerStatePacket other, double epsilon)
   {
      return (this.state == other.state);
   }
}
