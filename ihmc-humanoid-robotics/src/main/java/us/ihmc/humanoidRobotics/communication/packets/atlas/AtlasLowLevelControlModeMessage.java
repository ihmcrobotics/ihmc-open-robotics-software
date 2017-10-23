package us.ihmc.humanoidRobotics.communication.packets.atlas;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
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

   public AtlasLowLevelControlModeMessage(ControlMode request)
   {
      requestedControlMode = request;
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

   public AtlasLowLevelControlModeMessage(Random random)
   {
      requestedControlMode = RandomNumbers.nextEnum(random, ControlMode.class);
   }
}
