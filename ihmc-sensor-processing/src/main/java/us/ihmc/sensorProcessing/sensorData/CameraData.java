package us.ihmc.sensorProcessing.sensorData;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.communication.producers.VideoSource;

public class CameraData
{
   public final VideoSource videoSource;
   public final BufferedImage image;
   public final long timestamp;
   public final CameraPinholeBrown intrinsicParameters;

   public CameraData(VideoSource videoSource, BufferedImage image, long timestamp, CameraPinholeBrown intrinsicParameters)
   {
      this.videoSource = videoSource;
      this.image = image;
      this.timestamp = timestamp;
      this.intrinsicParameters = intrinsicParameters;
   }
}