package us.ihmc.sensorProcessing.sensorData;

import java.awt.image.BufferedImage;

import us.ihmc.communication.producers.VideoSource;

@Deprecated
public class CameraData
{
   public final VideoSource videoSource;
   public final BufferedImage image;
   public final long timestamp;
   public final Object intrinsicParameters;

   public CameraData(VideoSource videoSource, BufferedImage image, long timestamp, Object intrinsicParameters)
   {
      this.videoSource = videoSource;
      this.image = image;
      this.timestamp = timestamp;
      this.intrinsicParameters = intrinsicParameters;
   }
}