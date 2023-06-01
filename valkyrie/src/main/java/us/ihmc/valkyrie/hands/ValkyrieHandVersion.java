package us.ihmc.valkyrie.hands;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.valkyrie.hands.athena.AthenaFingerlessHandModel;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel;
import us.ihmc.valkyrie.hands.psyonic.PsyonicHandModel;
import us.ihmc.valkyrie.hands.zimmer.ZimmerGripperModel;

public enum ValkyrieHandVersion
{
   Athena, Psyonic, Zimmer, MassSim, AthenaFingerless, None;

   public static final String LEFT_HAND_VERSION_ENVIRONMENT_VARIABLE_NAME = "IHMC_VALKYRIE_LEFT_HAND_VERSION";
   public static final String RIGHT_HAND_VERSION_ENVIRONMENT_VARIABLE_NAME = "IHMC_VALKYRIE_RIGHT_HAND_VERSION";

   public static ValkyrieHandVersion fromEnvironment(RobotSide robotSide)
   {
      String varName = robotSide == RobotSide.LEFT ? LEFT_HAND_VERSION_ENVIRONMENT_VARIABLE_NAME : RIGHT_HAND_VERSION_ENVIRONMENT_VARIABLE_NAME;
      String valueFromEnvironment = System.getenv(varName);

      if (valueFromEnvironment == null)
         return null;

      for (ValkyrieHandVersion version : values())
      {
         if (valueFromEnvironment.trim().toLowerCase().contains(version.name().toLowerCase()))
            return version;
      }

      throw new IllegalStateException("Unhandled variable value for %s hand: %s".formatted(robotSide, valueFromEnvironment));
   }

   public ValkyrieHandModel getHandModel()
   {
      switch (this)
      {
         case Athena:
            return new AthenaHandModel();
         case Psyonic:
            return new PsyonicHandModel();
         case Zimmer:
            return new ZimmerGripperModel();
         case MassSim:
            return new ArmMassSimModel();
         case AthenaFingerless:
            return new AthenaFingerlessHandModel();
         case None:
            return null;
         default:
            throw new IllegalStateException("Unexpected hand type: " + this);
      }
   }

   public static ValkyrieHandVersion getHandVersion(RobotSide robotSide, RobotDefinition robotDefinition)
   {
      if (AthenaHandModel.hasAthenaHand(robotSide, robotDefinition))
         return Athena;
      if (PsyonicHandModel.hasPsyonicHand(robotSide, robotDefinition))
         return Psyonic;
      if (ZimmerGripperModel.hasZimmerGripper(robotSide, robotDefinition))
         return Zimmer;
      if (ArmMassSimModel.hasArmMassSim(robotSide, robotDefinition))
         return MassSim;
      if (AthenaFingerlessHandModel.hasAthenaFingerlessHand(robotSide, robotDefinition))
         return AthenaFingerless;
      return None;
   }
}
