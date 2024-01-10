package us.ihmc.perception.opencv;

import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.perception.BytedecoImage;

public class OpenCVArUcoMakerDetectionSwapData
{
   private BytedecoImage rgb8ImageForDetection;
   private boolean hasDetected = false;
   private final OpenCVArUcoMarkerDetectionOutput output = new OpenCVArUcoMarkerDetectionOutput();

   public OpenCVArUcoMakerDetectionSwapData()
   {
   }

   public void ensureImageInitialized(int imageWidth, int imageHeight)
   {
      if (rgb8ImageForDetection == null)
      {
         rgb8ImageForDetection = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      }
   }

   public BytedecoImage getRgb8ImageForDetection()
   {
      return rgb8ImageForDetection;
   }

   public OpenCVArUcoMarkerDetectionOutput getOutput()
   {
      return output;
   }

   public void setHasDetected(boolean hasDetected)
   {
      this.hasDetected = hasDetected;
   }

   public boolean getHasDetected()
   {
      return hasDetected;
   }
}
