package us.ihmc.robotDataCommunication.logger;

import java.net.InetAddress;

import us.ihmc.multicastLogDataProtocol.LogUtils;

public enum LogSettings
{
   ATLAS_IAN(true, "239.255.25.1", 5, 4, 0),
   ATLAS_NO_CAMERAS(true),
   VALKYRIE_IHMC(true, "239.255.25.2", 5, 4, 0),
   VALKYRIE_NO_CAMERAS(true),
   STEPPR_IHMC(true, "239.255.25.3", 2, 3),
   SIMULATION(false, "239.255.25.4"),
   BEHAVIOR(false),
   EXO_X1A(false),
   EXO_HOPPER(false),
   ETHERCAT(false),
   HAND(false),
   MINI_BEAST(false),
   BABY_BEAST(false);

   private final boolean log;
   private final int[] cameras;
   private final InetAddress videoStream;

   LogSettings(boolean log, int... cameras)
   {
      this(log, null, cameras);
   }

   LogSettings(boolean log, String videoStreamGroup, int... cameras)
   {
      this.log = log;
      this.cameras = cameras;
      if (videoStreamGroup == null)
      {
         this.videoStream = null;
      }
      else
      {
         this.videoStream = LogUtils.getByName(videoStreamGroup);
      }
   }

   public int[] getCameras()
   {
      return cameras;
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
