package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

public class LookAtStatus extends Packet<LookAtStatus>
{

   @Override
   public boolean epsilonEquals(LookAtStatus other, double epsilon)
   {
      return false;
   }

   public boolean isFinished()
   {
      return false;
   }

   public int getIndex()
   {
      return 0;
   }

}
