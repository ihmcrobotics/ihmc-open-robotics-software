package us.ihmc.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;

public class BytedecoOpenCVTools
{
   public static void clamp(Mat source, Mat destination, double min, double max)
   {
      int normType = opencv_core.NORM_MINMAX;
      int depthType = -1; // output array has the same type as src
      Mat mask = opencv_core.noArray(); // no operations
      opencv_core.normalize(source, destination, min, max, normType, depthType, mask);
   }

   public static void clamp32BitFloatTo8BitUnsignedChar(Mat source, Mat destination, double min, double max)
   {
      int normType = opencv_core.NORM_MINMAX;
      int depthType = opencv_core.CV_8U; // converting 32 bit to 8 bit
      Mat mask = opencv_core.noArray(); // no operations
      opencv_core.normalize(source, destination, min, max, normType, depthType, mask);
   }

   public static void convert8BitGrayTo8BitRGBA(Mat source, Mat destination)
   {
      int destinationChannels = 0; // automatic mode
      opencv_imgproc.cvtColor(source, destination, opencv_imgproc.COLOR_GRAY2RGBA, destinationChannels);
   }

   public static void scalePixelValues(Mat image, double scaleFactor)
   {
      double delta = 0.0; // no delta added
      int resultType = -1; // the output matrix will have the same type as the input
      image.convertTo(image, resultType, scaleFactor, delta);
   }

   public static void flipY(Mat source, Mat destination)
   {
      int flipCode = 0; // 0 flips X, 1 flips Y, -1 flips both
      opencv_core.flip(source, destination, flipCode);
   }

   public static void setRGBA8888ImageAlpha(Mat image, int alpha)
   {
      image.reshape(1, image.rows() * image.cols()).col(3).setTo(new Mat(new byte[] {(byte) alpha}));
   }
}
