package us.ihmc.sensorProcessing.sensorData;

import java.awt.image.BufferedImage;

import us.ihmc.robotics.robotSide.RobotSide;
import boofcv.struct.calib.IntrinsicParameters;

public class CameraData
{
   public final RobotSide robotSide;
   public final BufferedImage image;
   public final long timestamp;
   public final IntrinsicParameters intrinsicParameters;

   public CameraData(RobotSide robotSide, BufferedImage image, long timestamp, IntrinsicParameters intrinsicParameters)
   {
      this.robotSide = robotSide;
      this.image = image;
      this.timestamp = timestamp;
      this.intrinsicParameters = intrinsicParameters;
   }
}