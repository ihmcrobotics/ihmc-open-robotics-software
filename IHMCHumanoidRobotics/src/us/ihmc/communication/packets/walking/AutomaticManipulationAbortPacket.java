package us.ihmc.communication.packets.walking;

import us.ihmc.communication.packets.Packet;

public class AutomaticManipulationAbortPacket extends Packet<AutomaticManipulationAbortPacket>
{
   public boolean enable;

   public AutomaticManipulationAbortPacket()
   {
   }

   public AutomaticManipulationAbortPacket(boolean enable)
   {
      this.enable = enable;
   }

   public boolean getEnable()
   {
      return enable;
   }

   public void setEnable(boolean enable)
   {
      this.enable = enable;
   }

   @Override
   public boolean epsilonEquals(AutomaticManipulationAbortPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (enable != other.enable)
         return false;
      return true;
   }
}
