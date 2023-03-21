package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacv.Java2DFrameUtils;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import us.ihmc.log.LogTools;

import java.awt.image.BufferedImage;

public class BytedecoOpenCVTools
{
   public static final IntPointer compressionParametersPNG = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION);
   public static final IntPointer compressionParametersJPG = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

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

   public static void transferDepth16UC1ToLower8UC3(Mat source, Mat destination)
   {
      BytePointer data = source.data();

      Mat lower = new Mat(source.rows(), source.cols(), opencv_core.CV_8UC2, data);
      Mat upper = new Mat(source.rows(), source.cols(), opencv_core.CV_8UC1);
      upper.put(new Scalar(0));

      MatVector mats = new MatVector();
      mats.push_back(lower);
      mats.push_back(upper);

      opencv_core.merge(mats, destination);
   }

   public static void extractDepth16FromLower8UC3(Mat source, Mat destination)
   {
      MatVector mats = new MatVector();
      opencv_core.split(source, mats);

      LogTools.info("Previous Depth: {}", mats.size());

      MatVector finalMats = new MatVector();
      finalMats.push_back(mats.get(0));
      finalMats.push_back(mats.get(2));

      LogTools.info("New Depth: {}", mats.size());

      Mat depth8UC2 = new Mat(source.rows(), source.cols(), opencv_core.CV_8UC2);
      opencv_core.merge(finalMats, depth8UC2);
      Mat depth = new Mat(source.rows(), source.cols(), opencv_core.CV_16UC1, depth8UC2.data());

      destination.put(depth);
   }

   /**
    * Not an incorrect name, but overspecified for a general operation.
    */
   public static void convert8BitGrayTo8BitRGBA(Mat source, Mat destination)
   {
      convertGrayToRGBA(source, destination);
   }

   public static void convertGrayToRGBA(Mat source, Mat destination)
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
      opencv_imgproc.GaussianBlur(sourceImage, destinationImage, gaussianKernelSize, sigmaX, sigmaY, borderType);
   }

   public static Mat convertBufferedImageToMat(BufferedImage image)
   {
      return Java2DFrameUtils.toMat(image);
   }

   public static void compressRGBImageJPG(Mat image, Mat yuvImageToPack, BytePointer compressedBytes)
   {
      opencv_imgproc.cvtColor(image, yuvImageToPack, opencv_imgproc.COLOR_RGB2YUV_I420);
      opencv_imgcodecs.imencode(".jpg", yuvImageToPack, compressedBytes, compressionParametersJPG);
   }

   /* Not recommended for lossless use cases. */
   public static void compressDepthJPG(Mat image, BytePointer compressedBytes)
   {
      Mat depthRGBAMat = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC4, image.data());
      opencv_imgcodecs.imencode(".jpg", depthRGBAMat, compressedBytes, compressionParametersJPG);
   }

   /* Not recommended for lossless use cases. */
   public static void decompressImageJPG(byte[] data, Mat image)
   {
      BytePointer dataPointer = new BytePointer(data);
      Mat inputJPEGMat = new Mat(1, data.length, opencv_core.CV_8UC1, dataPointer);

      Mat depthRGBA8Mat = new Mat();
      opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, depthRGBA8Mat);

      Mat depthImage32FC1 = new Mat(depthRGBA8Mat.rows(), depthRGBA8Mat.cols(), opencv_core.CV_32FC1, depthRGBA8Mat);

      image.rows(depthRGBA8Mat.rows());
      image.cols(depthRGBA8Mat.cols());
      image.data(depthImage32FC1.data());
   }

   /* Not recommended for lossless use cases. */
   public static void decompressJPG(byte[] data, Mat dst)
   {
      BytePointer dataPointer = new BytePointer(data);
      Mat inputJPEGMat = new Mat(1, data.length, opencv_core.CV_8UC1, dataPointer);
      opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, dst);
   }

   public static void compressImagePNG(Mat image, BytePointer data)
   {
      opencv_imgcodecs.imencode(".png", image, data, compressionParametersPNG);
   }

   public static void decompressDepthPNG(BytePointer bytePointer, Mat image)
   {
      Mat compressedMat = new Mat(1, (int) bytePointer.limit(), opencv_core.CV_8UC1, bytePointer);
      opencv_imgcodecs.imdecode(compressedMat, opencv_imgcodecs.IMREAD_UNCHANGED, image);
   }

   public static Mat decompressImageJPGUsingYUV(BytePointer messageEncodedBytePointer)
   {
      Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
      Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);

      inputJPEGMat.cols((int) messageEncodedBytePointer.limit());
      inputJPEGMat.data(messageEncodedBytePointer);

      // imdecode takes the longest by far out of all this stuff
      opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, inputYUVI420Mat);

      Mat outputMat = new Mat((int) (inputYUVI420Mat.rows() / 1.5f), inputYUVI420Mat.cols(), opencv_core.CV_8UC4);
      opencv_imgproc.cvtColor(inputYUVI420Mat, outputMat, opencv_imgproc.COLOR_YUV2RGBA_I420);
      opencv_imgproc.cvtColor(outputMat, outputMat, opencv_imgproc.COLOR_RGBA2RGB);

      return outputMat;
   }

   public static void convertFloatToShort(Mat metricDepth, Mat shortDepthToPack, double scale, double delta)
   {
      metricDepth.convertTo(shortDepthToPack, opencv_core.CV_16UC1, scale, delta);
   }

   public static boolean dimensionsMatch(BytedecoImage a, BytedecoImage b)
   {
      return a.getImageWidth() == b.getImageWidth() && a.getImageHeight() == b.getImageHeight();
   }
}
