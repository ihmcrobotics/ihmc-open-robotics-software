package us.ihmc.avatar.stereoVision;

import us.ihmc.avatar.stereoVision.logging.StereoVisionLogger;
import us.ihmc.avatar.stereoVision.net.udp.StereoVisionUDPClient;
import us.ihmc.avatar.stereoVision.sensor.StereoVisionSensorReader;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class StereoVisionPublisher
{
   private final StereoVisionUDPClient udpClient;
   private final StereoVisionLogger stereoVisionLogger;

   public StereoVisionPublisher()
   {
      udpClient = new StereoVisionUDPClient();
      stereoVisionLogger = new StereoVisionLogger();
   }

   public void start()
   {
      SideDependentList<String> serialNumbers = new SideDependentList<>();
      serialNumbers.put(RobotSide.LEFT, "17403057");
      serialNumbers.put(RobotSide.RIGHT, "17372478");

      ImageCropInfo imageCropInfo = new ImageCropInfo();

      getSensorReader().start(serialNumbers, imageCropInfo);
      getSensorReader().registerListener(udpClient);
      getSensorReader().registerListener(stereoVisionLogger);
   }

   public void stop()
   {
      udpClient.stop();

      getSensorReader().unregisterListener(udpClient);
      getSensorReader().unregisterListener(stereoVisionLogger);
      getSensorReader().stop();
   }

   public StereoVisionUDPClient getUDPClient()
   {
      return udpClient;
   }

   public abstract StereoVisionSensorReader getSensorReader();
}
