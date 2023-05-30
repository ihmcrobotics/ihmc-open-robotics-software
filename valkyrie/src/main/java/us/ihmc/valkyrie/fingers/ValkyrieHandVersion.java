package us.ihmc.valkyrie.fingers;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.fingers.psyonicHand.PsyonicHandModel;
import us.ihmc.valkyrie.fingers.valkyrieHand.ValkyrieHandModel;
import us.ihmc.valkyrie.fingers.zimmerGripper.ZimmerGripperModel;

public enum ValkyrieHandVersion
{
   Valkyrie, Psyonic, Zimmer, None;

   public static final String LEFT_HAND_VERSION_ENVIRONMENT_VARIABLE_NAME = "IHMC_VALKYRIE_LEFT_HAND_VERSION";
   public static final String RIGHT_HAND_VERSION_ENVIRONMENT_VARIABLE_NAME = "IHMC_VALKYRIE_RIGHT_HAND_VERSION";

   public static ValkyrieHandVersion fromEnvironment(RobotSide robotSide)
   {
      String varName = robotSide == RobotSide.LEFT ? LEFT_HAND_VERSION_ENVIRONMENT_VARIABLE_NAME : RIGHT_HAND_VERSION_ENVIRONMENT_VARIABLE_NAME;
      String valueFromEnvironment = System.getenv(varName);

      if (valueFromEnvironment == null)
         return null;
      else if (valueFromEnvironment.trim().toLowerCase().contains(Valkyrie.name().toLowerCase()))
         return Valkyrie;
      else if (valueFromEnvironment.trim().toLowerCase().contains(Psyonic.name().toLowerCase()))
         return Psyonic;
      else if (valueFromEnvironment.trim().toLowerCase().contains(Zimmer.name().toLowerCase()))
         return Zimmer;
      throw new IllegalStateException("Unhandled variable value for %s hand: %s".formatted(robotSide, valueFromEnvironment));
   }

   public HandModel getHandModel()
   {
      switch (this)
      {
         case Valkyrie:
            return new ValkyrieHandModel();
         case Psyonic:
            return new PsyonicHandModel();
         case Zimmer:
            return new ZimmerGripperModel();
         case None:
            return null;
         default:
            throw new IllegalStateException("Unexpected hand type: " + this);
      }
   }
}
