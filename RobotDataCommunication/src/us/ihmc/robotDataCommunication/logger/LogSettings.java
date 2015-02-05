package us.ihmc.robotDataCommunication.logger;

import java.net.InetAddress;

import us.ihmc.multicastLogDataProtocol.LogUtils;

public enum LogSettings
{
   ATLAS_IAN("10.66.171.20", true, "239.255.25.1", 4, 5),
   VALKYRIE_IHMC("139.169.44.20", true, "239.255.25.2", 0, 1),
   STEPPR_IHMC("10.66.171.20", true, "239.255.25.3", 2, 3),
   SIMULATION("localhost", false, "239.255.25.4"),
   BEHAVIOR("139.169.44.20", false),
   EXO_X1A("192.168.1.5", false),
   EXO_HOPPER("192.168.1.5", false),
   ETHERCAT("192.168.1.5", false),
   HAND("localhost", false);

   private final boolean log;
   private final int[] cameras;
   private final InetAddress videoStream;
   private final String hostToBindTo;

   LogSettings(String hostToBindTo, boolean log, int... cameras)
   {
      this(hostToBindTo, log, null, cameras);
   }

   LogSettings(String hostToBindTo, boolean log, String videoStreamGroup, int... cameras)
   {
      this.log = log;
      this.cameras = cameras;
      this.hostToBindTo = hostToBindTo;
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

   public String getHost()
   {
      return hostToBindTo;
   }
}
