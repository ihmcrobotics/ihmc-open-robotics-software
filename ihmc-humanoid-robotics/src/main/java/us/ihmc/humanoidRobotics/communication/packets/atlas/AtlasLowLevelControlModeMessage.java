package us.ihmc.humanoidRobotics.communication.packets.atlas;

import us.ihmc.communication.packets.Packet;

public class AtlasLowLevelControlModeMessage extends Packet<AtlasLowLevelControlModeMessage>
{
   public enum ControlMode
   {
      STAND_PREP, FREEZE;
   }

   public ControlMode requestedControlMode;

   public AtlasLowLevelControlModeMessage()
   {
   }

   @Override
   public void set(AtlasLowLevelControlModeMessage other)
   {
      setPacketInformation(other);
      requestedControlMode = other.requestedControlMode;
   }

   public void setRequestedControlMode(ControlMode requestedControlMode)
   {
      this.requestedControlMode = requestedControlMode;
   }

   public ControlMode getRequestedControlMode()
   {
      return requestedControlMode;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      String errorMessage = null;
      if (requestedControlMode == null)
         errorMessage = "The field requestedControlMode is null.";
      return errorMessage;
   }

   @Override
   public boolean epsilonEquals(AtlasLowLevelControlModeMessage other, double epsilon)
   {
      return requestedControlMode == other.requestedControlMode;
   }
}
