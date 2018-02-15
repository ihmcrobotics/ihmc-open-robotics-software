package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.communication.packets.Packet;

public class QuadrupedForceControllerStatePacket extends Packet<QuadrupedForceControllerStatePacket>
{
   public QuadrupedForceControllerState state;

   public QuadrupedForceControllerStatePacket()
   {
      this.state = null;
   }

   public QuadrupedForceControllerStatePacket(QuadrupedForceControllerState state)
   {
      this.state = state;
   }

   @Override
   public void set(QuadrupedForceControllerStatePacket other)
   {
      state = other.state;
      setPacketInformation(other);
   }

   public QuadrupedForceControllerState get()
   {
      return state;
   }

   public void set(QuadrupedForceControllerState state)
   {
      this.state = state;
   }

   @Override
   public boolean epsilonEquals(QuadrupedForceControllerStatePacket other, double epsilon)
   {
      return (this.state == other.state);
   }
}
