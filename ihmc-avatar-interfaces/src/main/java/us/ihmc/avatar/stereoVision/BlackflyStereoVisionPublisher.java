package us.ihmc.avatar.stereoVision;

import us.ihmc.avatar.stereoVision.sensor.StereoVisionBlackflyReader;
import us.ihmc.avatar.stereoVision.sensor.StereoVisionSensorReader;
import us.ihmc.commons.thread.ThreadTools;

public class BlackflyStereoVisionPublisher extends StereoVisionPublisher
{
   private final StereoVisionBlackflyReader blackflyReader;

   public BlackflyStereoVisionPublisher()
   {
      // Setup Blackfly reader
      blackflyReader = new StereoVisionBlackflyReader();
   }

   @Override
   public StereoVisionSensorReader getSensorReader()
   {
      return blackflyReader;
   }

   public static void main(String[] args)
   {
      BlackflyStereoVisionPublisher blackflyStereoVisionPublisher = new BlackflyStereoVisionPublisher();
      blackflyStereoVisionPublisher.start();

      Runtime.getRuntime().addShutdownHook(new Thread(blackflyStereoVisionPublisher::stop));

      ThreadTools.sleepForever();
   }
}
