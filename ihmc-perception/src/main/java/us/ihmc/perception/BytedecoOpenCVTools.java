package us.ihmc.perception;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;

public class BytedecoOpenCVTools
{
   public static final int FLIP_Y = 0;
   public static final int FLIP_X = 1;
   public static final int FLIP_BOTH = -1;

   public static int getImageWidth(Mat image)
   {
      return image.cols();
   }

   public static int getImageHeight(Mat image)
   {
      return image.rows();
   }

   public static void clamp(Mat source, Mat destination, double min, double max)
   {
      int normType = opencv_core.NORM_MINMAX;
      int depthType = -1; // output array has the same type as src
      Mat mask = opencv_core.noArray(); // no operations
      opencv_core.normalize(source, destination, min, max, normType, depthType, mask);
   }

   public static void clampTo8BitUnsignedChar(Mat source, Mat destination, double min, double max)
   {
      int normType = opencv_core.NORM_MINMAX;
      int depthType = opencv_core.CV_8U; // converting to 8 bit
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
      int flipCode = FLIP_Y;
      opencv_core.flip(source, destination, flipCode);
   }

   public static void setRGBA8888ImageAlpha(Mat image, int alpha)
   {
      image.reshape(1, image.rows() * image.cols()).col(3).setTo(new Mat(new byte[] {(byte) alpha}));
   }

   public static void convertABGRToRGBA(Mat srcABGR, Mat dstRGBA)
   {
      IntPointer fromABGRToRGBA = new IntPointer(0, 3, 1, 2, 2, 1, 3, 0);
      opencv_core.mixChannels(srcABGR, 1, dstRGBA, 1, fromABGRToRGBA, 4);
   }

   public static void convertRGBAToABGR(Mat srcRGBA, Mat dstABGR)
   {
      IntPointer fromABGRToRGBA = new IntPointer(0, 3, 1, 2, 2, 1, 3, 0);
      opencv_core.mixChannels(srcRGBA, 1, dstABGR, 1, fromABGRToRGBA, 4);
   }

   public static void blur(Mat sourceImage, Mat destinationImage)
   {
      int gaussianSize = 6;
      int size = gaussianSize * 2 + 1;
      Size gaussianKernelSize = new Size();
      gaussianKernelSize.width(size);
      gaussianKernelSize.height(size);
      double sigmaX = 4.74;
      double sigmaY = sigmaX;
      int borderType = opencv_core.BORDER_DEFAULT;
      opencv_imgproc.GaussianBlur(sourceImage,
                                  destinationImage,
                                  gaussianKernelSize,
                                  sigmaX,
                                  sigmaY,
                                  borderType);
   }

   public static String getTypeString(int type)
   {
      int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

      int enum_ints[] =       {opencv_core.CV_8U,  opencv_core.CV_8UC1,  opencv_core.CV_8UC2,  opencv_core.CV_8UC3,  opencv_core.CV_8UC4,
                               opencv_core.CV_8S,  opencv_core.CV_8SC1,  opencv_core.CV_8SC2,  opencv_core.CV_8SC3,  opencv_core.CV_8SC4,
                               opencv_core.CV_16U, opencv_core.CV_16UC1, opencv_core.CV_16UC2, opencv_core.CV_16UC3, opencv_core.CV_16UC4,
                               opencv_core.CV_16S, opencv_core.CV_16SC1, opencv_core.CV_16SC2, opencv_core.CV_16SC3, opencv_core.CV_16SC4,
                               opencv_core.CV_32S, opencv_core.CV_32SC1, opencv_core.CV_32SC2, opencv_core.CV_32SC3, opencv_core.CV_32SC4,
                               opencv_core.CV_32F, opencv_core.CV_32FC1, opencv_core.CV_32FC2, opencv_core.CV_32FC3, opencv_core.CV_32FC4,
                               opencv_core.CV_64F, opencv_core.CV_64FC1, opencv_core.CV_64FC2, opencv_core.CV_64FC3, opencv_core.CV_64FC4};

      String enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
                               "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
                               "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
                               "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
                               "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
                               "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
                               "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

      for (int i = 0; i < numImgTypes; i++)
      {
         if (type == enum_ints[i])
            return enum_strings[i];
      }
      return "unknown image type";
   }
}
