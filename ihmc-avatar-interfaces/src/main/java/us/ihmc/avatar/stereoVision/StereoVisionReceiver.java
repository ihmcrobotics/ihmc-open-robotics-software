package us.ihmc.avatar.stereoVision;

import us.ihmc.avatar.stereoVision.net.udp.StereoVisionUDPServer;
import us.ihmc.commons.thread.ThreadTools;

public class StereoVisionReceiver
{
   private final StereoVisionUDPServer udpServer;

   public StereoVisionReceiver()
   {
      udpServer = new StereoVisionUDPServer();
   }

   public void start()
   {
      udpServer.start("127.0.0.1");
   }

   public static void main(String[] args)
   {
      StereoVisionReceiver stereoVisionReceiver = new StereoVisionReceiver();
      stereoVisionReceiver.start();

      ThreadTools.sleepForever();
   }
}
