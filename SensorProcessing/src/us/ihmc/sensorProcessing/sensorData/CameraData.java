package us.ihmc.sensorProcessing.sensorData;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.robotics.robotSide.RobotSide;

public class CameraData
{
   public final RobotSide robotSide;
   public final VideoSource videoSource;
   public final BufferedImage image;
   public final long timestamp;
   public final IntrinsicParameters intrinsicParameters;

   public CameraData(RobotSide robotSide, VideoSource videoSource, BufferedImage image, long timestamp, IntrinsicParameters intrinsicParameters)
   {
      this.robotSide = robotSide;
      this.videoSource = videoSource;
      this.image = image;
      this.timestamp = timestamp;
      this.intrinsicParameters = intrinsicParameters;
   }
}