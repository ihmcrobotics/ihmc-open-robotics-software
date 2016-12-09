package us.ihmc.robotDataLogger.logger;

import java.net.InetAddress;

import us.ihmc.multicastLogDataProtocol.LogUtils;

public enum LogSettings
{
   ATLAS_IAN(true, "239.255.25.1"),
   ATLAS_NO_CAMERAS(true),
   VALKYRIE_IHMC(true, "239.255.25.2"),
   VALKYRIE_JSC(true, "239.255.25.2"),
   VALKYRIE_NO_CAMERAS(true),
   STEPPR_IHMC(true, "239.255.25.3"),
   SIMULATION(false, "239.255.25.4"),
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
   MEGABOTS(true, "239.255.25.5"),
   FOOTSTEP_PLANNER(true);

   private final boolean log;
   private final InetAddress videoStream;

   LogSettings(boolean log)
   {
      this(log, null);
   }

   LogSettings(boolean log, String videoStreamGroup)
   {
      this.log = log;
      if (videoStreamGroup == null)
      {
         this.videoStream = null;
      }
      else
      {
         this.videoStream = LogUtils.getByName(videoStreamGroup);
      }
   }

   public boolean isLog()
   {
      return log;
   }

   public InetAddress getVideoStream()
   {
      return videoStream;
   }
}
