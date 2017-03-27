package us.ihmc.robotDataLogger.logger;

public enum LogSettings
{
   ATLAS_IAN(true, "AtlasGUI"),
   ATLAS_NO_CAMERAS(true),
   VALKYRIE_IHMC(true, "ValkyrieIHMCGUI"),
   VALKYRIE_JSC(true, "ValkyrieJSCGUI"),
   VALKYRIE_NO_CAMERAS(true),
   STEPPR_IHMC(true, "StepprIHMCGUI"),
   SIMULATION(true, "SimulationGUI"),
   TEST_LOGGER(true),
   BEHAVIOR(true),
   TOOLBOX(false),
   EXO_X1A(false),
   EXO_HOPPER(false),
   ETHERCAT(false),
   HAND(false),
   MINI_BEAST(false),
   BABY_BEAST(true),
   V2EXO(true),
   MEGABOTS(true, "MegaBOTSGUI"),
   FOOTSTEP_PLANNER(true);

   private final boolean log;
   private final String videoStream;

   LogSettings(boolean log)
   {
      this(log, null);
   }

   LogSettings(boolean log, String videoStreamIdentifier)
   {
      this.log = log;
      this.videoStream = videoStreamIdentifier;
   }

   public boolean isLog()
   {
      return log;
   }

   public String getVideoStream()
   {
      return videoStream;
   }
}
