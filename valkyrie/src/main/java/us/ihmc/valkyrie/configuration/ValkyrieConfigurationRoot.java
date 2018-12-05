package us.ihmc.valkyrie.configuration;

public class ValkyrieConfigurationRoot
{
   public static final boolean VALKYRIE_WITH_ARMS = true;

   public static final String REAL_ROBOT_SDF_FILE;
   public static final String SIM_SDF_FILE;

   static
   {
      if (VALKYRIE_WITH_ARMS)
      {
         REAL_ROBOT_SDF_FILE = "models/val_description/sdf/valkyrie_sim.sdf";
         SIM_SDF_FILE = "models/val_description/sdf/valkyrie_sim.sdf";       
      }
      else
      {
         REAL_ROBOT_SDF_FILE = "models/val_description/sdf/valkyrie_sim_no_arms.sdf";
         SIM_SDF_FILE = "models/val_description/sdf/valkyrie_sim_no_arms.sdf";       
      }
   }
}
