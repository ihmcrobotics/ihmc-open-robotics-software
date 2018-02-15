package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;

public class AutomaticManipulationAbortMessage extends Packet<AutomaticManipulationAbortMessage>
{
   public boolean enable;

   public AutomaticManipulationAbortMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(AutomaticManipulationAbortMessage other)
   {
      setPacketInformation(other);
      enable = other.enable;
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
   public boolean epsilonEquals(AutomaticManipulationAbortMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (enable != other.enable)
         return false;
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      // No data besides a boolean, so always good.
      return null;
   }
}
