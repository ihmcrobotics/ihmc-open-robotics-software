package us.ihmc.avatar.stereoVision.sensor;

import us.ihmc.robotics.robotSide.RobotSide;

public interface StereoVisionSensorReaderListener
{
   void onNewImage(RobotSide side, int frameNumber, int width, int height, byte[] imageData);
}
