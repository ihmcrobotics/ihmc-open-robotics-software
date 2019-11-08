package us.ihmc.valkyrie.configuration;

public enum ValkyrieRobotVersion
{
   DEFAULT,
   ARM_MASS_SIM,
   ARMLESS;

   public String getRealRobotSdfFile()
   {
      switch(this)
      {
         case DEFAULT:
            return "models/val_description/sdf/valkyrie_sim.sdf";
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
         case ARM_MASS_SIM:
            return true;
         case ARMLESS:
            return false;
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public boolean hasHands()
   {
      switch(this)
      {
         case DEFAULT:
            return true;
         case ARM_MASS_SIM:
         case ARMLESS:
            return false;
         default:
            throw new RuntimeException("ValkyrieRobotVersion: Unimplemented enumeration case : " + this);
      }
   }
}
