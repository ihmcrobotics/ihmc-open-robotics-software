package us.ihmc.perception.tools;

import us.ihmc.perception.BytedecoImage;

import java.nio.FloatBuffer;

public class PerceptionDataTools
{
   public static FloatBuffer convertSphericalDepthImageToPointCloudInSensorFrame(BytedecoImage imageToConvert, double verticalFOV, double horizontalFOV)
   {
      int numberOfPoints = imageToConvert.getImageHeight() * imageToConvert.getImageWidth();
      FloatBuffer depthData = NativeMemoryTools.allocate(3 * Float.BYTES * numberOfPoints).asFloatBuffer();
      depthData.rewind();

      int rowMiddle = imageToConvert.getImageHeight() / 2;
      int colMiddle = imageToConvert.getImageWidth() / 2;
      for (int row = 0; row < imageToConvert.getImageHeight(); row++)
      {
         for (int col = 0; col < imageToConvert.getImageWidth(); col++)
         {
            int colFromCenter = -col - colMiddle;
            int rowFromCenter = -(row  - rowMiddle);

            double yaw = colFromCenter / (double) imageToConvert.getImageWidth() * horizontalFOV;
            double pitch = rowFromCenter / (double) imageToConvert.getImageHeight() * verticalFOV;

            int index = row * imageToConvert.getImageWidth() + col;
            double depth = imageToConvert.getPointerForAccessSpeed().getShort(index * 2L) / 1000.0;
            double r = depth * Math.cos(pitch);

            double x = r * Math.cos(yaw);
            double y = r * Math.sin(yaw);
            double z = depth * Math.sin(pitch);

            depthData.put((float) x);
            depthData.put((float) y);
            depthData.put((float) z);
         }
      }

      return depthData;
   }
}
