package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerEnum;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingStateEnum;

public class QuadrupedSteppingStatePacket extends Packet<QuadrupedSteppingStatePacket>
{
   private QuadrupedSteppingStateEnum state;

   public QuadrupedSteppingStatePacket()
   {
      this.state = null;
   }

   public QuadrupedSteppingStatePacket(QuadrupedSteppingStateEnum state)
   {
      this.state = state;
   }

   public QuadrupedSteppingStateEnum get()
   {
      return state;
   }

   public void set(QuadrupedSteppingStateEnum state)
   {
      this.state = state;
   }

   public void set(QuadrupedSteppingStatePacket other)
   {
      this.state = other.state;
   }

   @Override
   public boolean epsilonEquals(QuadrupedSteppingStatePacket other, double epsilon)
   {
      return (this.state == other.state);
   }
}
