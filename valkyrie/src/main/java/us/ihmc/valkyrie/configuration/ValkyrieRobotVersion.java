package us.ihmc.valkyrie.configuration;

import us.ihmc.avatar.drcRobot.RobotVersion;

public enum ValkyrieRobotVersion implements RobotVersion
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

   public String getRealRobotURDFFile()
   {
      switch(this)
      {
         case DEFAULT:
            return "models/val_description/urdf/valkyrie_sim.urdf";
         case FINGERLESS:
            return "models/val_description/urdf/valkyrie_sim_no_fingers.urdf";
         case ARM_MASS_SIM:
            return "models/val_description/urdf/valkyrie_sim_arm_mass_sim.urdf";
         case ARMLESS:
            return "models/val_description/urdf/valkyrie_sim_no_arms.urdf";
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public String getSimURDFFile()
   {
      switch(this)
      {
         case DEFAULT:
            return "models/val_description/urdf/valkyrie_sim.urdf";
         case FINGERLESS:
            return "models/val_description/urdf/valkyrie_sim_no_fingers.urdf";
         case ARM_MASS_SIM:
            return "models/val_description/urdf/valkyrie_sim_arm_mass_sim.urdf";
         case ARMLESS:
            return "models/val_description/urdf/valkyrie_sim_no_arms.urdf";
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public boolean hasBothArms()
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

   public boolean hasHands()
   {
      switch (this)
      {
         case DEFAULT:
         case FINGERLESS:
            return true;
         case ARM_MASS_SIM:
         case ARMLESS:
            return false;
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }
}
