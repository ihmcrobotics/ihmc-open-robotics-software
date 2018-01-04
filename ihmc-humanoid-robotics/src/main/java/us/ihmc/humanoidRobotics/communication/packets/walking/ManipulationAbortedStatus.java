package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.SettablePacket;

public class ManipulationAbortedStatus extends SettablePacket<ManipulationAbortedStatus>
{
   public ManipulationAbortedStatus()
   {
   }

   @Override
   public void set(ManipulationAbortedStatus other)
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
