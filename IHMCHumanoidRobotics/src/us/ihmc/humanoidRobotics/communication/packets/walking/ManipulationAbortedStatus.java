package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;

public class ManipulationAbortedStatus extends Packet<ManipulationAbortedStatus>
{
   public ManipulationAbortedStatus()
   {
   }

   @Override
   public boolean epsilonEquals(ManipulationAbortedStatus other, double epsilon)
   {
      if (other == null)
         return false;
      return true;
   }
}
