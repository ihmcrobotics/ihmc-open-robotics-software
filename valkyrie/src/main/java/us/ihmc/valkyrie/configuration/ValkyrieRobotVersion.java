package us.ihmc.valkyrie.configuration;

public enum ValkyrieRobotVersion
{
   DEFAULT,
   FINGERLESS,
   ARM_MASS_SIM,
   ARMLESS;

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
