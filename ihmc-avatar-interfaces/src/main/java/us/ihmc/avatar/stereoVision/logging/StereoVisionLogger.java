package us.ihmc.avatar.stereoVision.logging;

import us.ihmc.avatar.stereoVision.sensor.StereoVisionSensorReaderListener;
import us.ihmc.robotics.robotSide.RobotSide;

// TODO: implement
public class StereoVisionLogger implements StereoVisionSensorReaderListener
{
   @Override
   public void onNewImage(RobotSide side, int frameNumber, int width, int height, byte[] imageData)
   {
      System.out.println("StereoVisionLogger onNewImage");
   }
}
