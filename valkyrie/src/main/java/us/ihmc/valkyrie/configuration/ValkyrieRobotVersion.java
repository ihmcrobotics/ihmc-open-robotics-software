package us.ihmc.valkyrie.configuration;

public enum ValkyrieRobotVersion
{
   DEFAULT,
   FINGERLESS,
   ARM_MASS_SIM,
   ARMLESS;

   public static final String ROBOT_VERSION_ENVIRONMENT_VARIABLE_NAME = "IHMC_VALKYRIE_ROBOT_VERSION";

   public static ValkyrieRobotVersion fromEnvironment()
   {
      String valueFromEnvironment = System.getenv(ROBOT_VERSION_ENVIRONMENT_VARIABLE_NAME);

      if (valueFromEnvironment == null)
         return DEFAULT;
      else if (valueFromEnvironment.trim().toLowerCase().contains(FINGERLESS.name().toLowerCase()))
         return FINGERLESS;
      else if (valueFromEnvironment.trim().toLowerCase().contains(ARM_MASS_SIM.name().toLowerCase()))
         return ARM_MASS_SIM;
      else if (valueFromEnvironment.trim().toLowerCase().contains(ARMLESS.name().toLowerCase()))
         return ARMLESS;
      else
         return DEFAULT;
   }

   public String getRealRobotSdfFile()
   {
      switch(this)
      {
         case DEFAULT:
            return "models/val_description/sdf/valkyrie_sim.sdf";
         case FINGERLESS:
            return "models/val_description/sdf/valkyrie_sim_no_fingers.sdf";
         case ARM_MASS_SIM:
            return "models/val_description/sdf/valkyrie_sim_arm_mass_sim.sdf";
         case ARMLESS:
            return "models/val_description/sdf/valkyrie_sim_no_arms.sdf";
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public String getSimSdfFile()
   {
      switch(this)
      {
         case DEFAULT:
            return "models/val_description/sdf/valkyrie_sim.sdf";
         case FINGERLESS:
            return "models/val_description/sdf/valkyrie_sim_no_fingers.sdf";
         case ARM_MASS_SIM:
            return "models/val_description/sdf/valkyrie_sim_arm_mass_sim.sdf";
         case ARMLESS:
            return "models/val_description/sdf/valkyrie_sim_no_arms.sdf";
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public boolean hasArms()
   {
      switch(this)
      {
         case DEFAULT:
         case FINGERLESS:
         case ARM_MASS_SIM:
            return true;
         case ARMLESS:
            return false;
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public boolean hasFingers()
   {
      switch(this)
      {
         case DEFAULT:
            return true;
         case FINGERLESS:
         case ARM_MASS_SIM:
         case ARMLESS:
            return false;
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }
}
