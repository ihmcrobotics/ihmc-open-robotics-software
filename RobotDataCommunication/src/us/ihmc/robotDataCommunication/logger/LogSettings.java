package us.ihmc.robotDataCommunication.logger;

public enum LogSettings
{
   ATLAS_IAN(true, 4, 5),
   VALKYRIE_IHMC(true, 0, 1),
   STEPPR_IHMC(true, 2, 3),
   SIMULATION(false),
   BEHAVIOR(false), 
   EXO_X1A(false),
   EXO_HOPPER(false), 
   ETHERCAT(false);

   
   private final boolean log;
   private final int[] cameras;
   
   LogSettings(boolean log, int... cameras)
   {
      this.log = log;
      this.cameras = cameras;
   }

   public int[] getCameras()
   {
      return cameras;
   }

   public boolean isLog()
   {
      return log;
   }
}
