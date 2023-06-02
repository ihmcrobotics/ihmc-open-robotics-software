package us.ihmc.valkyrie.configuration;

import us.ihmc.avatar.drcRobot.RobotVersion;

public enum ValkyrieRobotVersion implements RobotVersion
{
   DEFAULT, FINGERLESS, ARMLESS, DUAL_ARM_MASS_SIM, DUAL_PSYONIC, DUAL_ZIMMER;

   public static final String ROBOT_VERSION_ENVIRONMENT_VARIABLE_NAME = "IHMC_VALKYRIE_ROBOT_VERSION";

   public static ValkyrieRobotVersion fromEnvironment()
   {
      String valueFromEnvironment = System.getenv(ROBOT_VERSION_ENVIRONMENT_VARIABLE_NAME);

      if (valueFromEnvironment == null)
         return DEFAULT;

      for (ValkyrieRobotVersion version : values())
      {
         if (valueFromEnvironment.trim().toLowerCase().contains(version.name().toLowerCase()))
            return version;
      }

      return DEFAULT;
   }

   public String getURDFFile()
   {
      switch (this)
      {
         case DEFAULT:
            return "models/valkyrie_dual_athena.urdf";
         case FINGERLESS:
            return "models/valkyrie_dual_athena_no_fingers.urdf";
         case ARMLESS:
            return "models/valkyrie_no_arms.urdf";
         case DUAL_ARM_MASS_SIM:
            return "models/valkyrie_dual_arm_mass_sim.urdf";
         case DUAL_PSYONIC:
            return "models/valkyrie_dual_psyonic.urdf";
         case DUAL_ZIMMER:
            return "models/valkyrie_dual_zimmer.urdf";
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public boolean hasArms()
   {
      switch (this)
      {
         case DEFAULT:
         case FINGERLESS:
         case DUAL_ARM_MASS_SIM:
         case DUAL_PSYONIC:
         case DUAL_ZIMMER:
            return true;
         case ARMLESS:
            return false;
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   @Deprecated // Shoulder be looking at the actual robot definition
   public boolean hasFingers()
   {
      switch (this)
      {
         case DEFAULT:
         case DUAL_PSYONIC:
         case DUAL_ZIMMER:
            return true;
         case FINGERLESS:
         case ARMLESS:
         case DUAL_ARM_MASS_SIM:
            return false;
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   @Deprecated // Shoulder be looking at the actual robot definition
   public boolean hasHands()
   {
      switch (this)
      {
         case DEFAULT:
         case FINGERLESS:
         case DUAL_PSYONIC:
         case DUAL_ZIMMER:
            return true;
         case DUAL_ARM_MASS_SIM:
         case ARMLESS:
            return false;
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }
}
