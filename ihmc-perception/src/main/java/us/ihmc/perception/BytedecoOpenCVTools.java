package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacv.Java2DFrameUtils;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.log.LogTools;

import java.awt.image.BufferedImage;

import static org.bytedeco.opencv.global.opencv_core.*;
import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

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

      Mat lower = new Mat(source.rows(), source.cols(), CV_8UC2, data);
      Mat upper = new Mat(source.rows(), source.cols(), CV_8UC1);
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

      Mat depth8UC2 = new Mat(source.rows(), source.cols(), CV_8UC2);
      opencv_core.merge(finalMats, depth8UC2);
      Mat depth = new Mat(source.rows(), source.cols(), CV_16UC1, depth8UC2.data());

      destination.put(depth);
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
      Mat depthRGBAMat = new Mat(image.rows(), image.cols(), CV_8UC4, image.data());
      opencv_imgcodecs.imencode(".jpg", depthRGBAMat, compressedBytes, compressionParametersJPG);
   }

   /* Not recommended for lossless use cases. */
   public static void decompressImageJPG(byte[] data, Mat image)
   {
      BytePointer dataPointer = new BytePointer(data);
      Mat inputJPEGMat = new Mat(1, data.length, CV_8UC1, dataPointer);

      Mat depthRGBA8Mat = new Mat();
      opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, depthRGBA8Mat);

      Mat depthImage32FC1 = new Mat(depthRGBA8Mat.rows(), depthRGBA8Mat.cols(), CV_32FC1, depthRGBA8Mat);

      image.rows(depthRGBA8Mat.rows());
      image.cols(depthRGBA8Mat.cols());
      image.data(depthImage32FC1.data());
   }

   /* Not recommended for lossless use cases. */
   public static void decompressJPG(byte[] data, Mat dst)
   {
      BytePointer dataPointer = new BytePointer(data);
      Mat inputJPEGMat = new Mat(1, data.length, CV_8UC1, dataPointer);
      opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, dst);
   }

   public static void compressImagePNG(Mat image, BytePointer data)
   {
      Mat depth = null;
      if (image.type() == opencv_core.CV_32FC1)
      {
         depth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC4, image.data());
      }
      else
      {
         depth = image;
      }
      opencv_imgcodecs.imencode(".png", depth, data, compressionParametersPNG);
   }

   public static void decompressDepthPNG(byte[] data, Mat image)
   {
      Mat compressedMat = new Mat(1, data.length, CV_8UC1, new BytePointer(data));
      opencv_imgcodecs.imdecode(compressedMat, opencv_imgcodecs.IMREAD_UNCHANGED, image);
   }

   public static String getTypeString(int type)
   {
      int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

      int enum_ints[] = {opencv_core.CV_8U,
                         opencv_core.CV_8UC1,
                         opencv_core.CV_8UC2,
                         opencv_core.CV_8UC3,
                         opencv_core.CV_8UC4,
                         opencv_core.CV_8S,
                         opencv_core.CV_8SC1,
                         opencv_core.CV_8SC2,
                         opencv_core.CV_8SC3,
                         opencv_core.CV_8SC4,
                         opencv_core.CV_16U,
                         opencv_core.CV_16UC1,
                         opencv_core.CV_16UC2,
                         opencv_core.CV_16UC3,
                         opencv_core.CV_16UC4,
                         opencv_core.CV_16S,
                         opencv_core.CV_16SC1,
                         opencv_core.CV_16SC2,
                         opencv_core.CV_16SC3,
                         opencv_core.CV_16SC4,
                         opencv_core.CV_32S,
                         opencv_core.CV_32SC1,
                         opencv_core.CV_32SC2,
                         opencv_core.CV_32SC3,
                         opencv_core.CV_32SC4,
                         opencv_core.CV_32F,
                         opencv_core.CV_32FC1,
                         opencv_core.CV_32FC2,
                         opencv_core.CV_32FC3,
                         opencv_core.CV_32FC4,
                         opencv_core.CV_64F,
                         opencv_core.CV_64FC1,
                         opencv_core.CV_64FC2,
                         opencv_core.CV_64FC3,
                         opencv_core.CV_64FC4};

      String enum_strings[] = {"CV_8U",
                               "CV_8UC1",
                               "CV_8UC2",
                               "CV_8UC3",
                               "CV_8UC4",
                               "CV_8S",
                               "CV_8SC1",
                               "CV_8SC2",
                               "CV_8SC3",
                               "CV_8SC4",
                               "CV_16U",
                               "CV_16UC1",
                               "CV_16UC2",
                               "CV_16UC3",
                               "CV_16UC4",
                               "CV_16S",
                               "CV_16SC1",
                               "CV_16SC2",
                               "CV_16SC3",
                               "CV_16SC4",
                               "CV_32S",
                               "CV_32SC1",
                               "CV_32SC2",
                               "CV_32SC3",
                               "CV_32SC4",
                               "CV_32F",
                               "CV_32FC1",
                               "CV_32FC2",
                               "CV_32FC3",
                               "CV_32FC4",
                               "CV_64F",
                               "CV_64FC1",
                               "CV_64FC2",
                               "CV_64FC3",
                               "CV_64FC4"};

      for (int i = 0; i < numImgTypes; i++)
      {
         if (type == enum_ints[i])
            return enum_strings[i];
      }
      return "unknown image type";
   }

   public static void printMat(Mat image, String prefix)
   {
      LogTools.info(matToString(image, prefix));
   }

   public static String matToString(Mat image, String prefix)
   {
      StringBuilder matString = new StringBuilder("Mat: [" + prefix + "] \n");

      for (int i = 0; i < image.rows(); i++)
      {
         for (int j = 0; j < image.cols(); j++)
         {
            matString.append(image.ptr(i, j).getShort()).append("\t");
         }
         matString.append("\n");
      }

      return matString.toString();
   }

   public static void displayVideoPacketDepth(VideoPacket videoPacket)
   {
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16UC1);
      byte[] compressedByteArray = videoPacket.getData().toArray();
      BytedecoOpenCVTools.decompressDepthPNG(compressedByteArray, depthImage);

      Mat displayDepth = new Mat(depthImage.rows(), depthImage.cols(), opencv_core.CV_8UC1);
      Mat finalDisplayDepth = new Mat(depthImage.rows(), depthImage.cols(), opencv_core.CV_8UC3);

      BytedecoOpenCVTools.clampTo8BitUnsignedChar(depthImage, displayDepth, 0.0, 255.0);
      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

      imshow("/l515/depth", finalDisplayDepth);
      int code = waitKeyEx(30);
      if (code == 113)
      {
         System.exit(0);
      }
   }

   public static void displayVideoPacketColor(VideoPacket videoPacket)
   {
      Mat colorImage = new Mat(videoPacket.getImageHeight(), videoPacket.getImageWidth(), opencv_core.CV_8UC3);
      byte[] compressedByteArray = videoPacket.getData().toArray();
      BytedecoOpenCVTools.decompressJPG(compressedByteArray, colorImage);

      imshow("Color Image", colorImage);
      int code = waitKeyEx(1);
      if (code == 113)
      {
         System.exit(0);
      }
   }

   public static void fillVideoPacket(BytePointer compressedBytes, byte[] heapArray, VideoPacket packet, int height, int width, long nanoTime)
   {
      compressedBytes.asBuffer().get(heapArray, 0, compressedBytes.asBuffer().remaining());
      packet.setTimestamp(nanoTime);
      packet.getData().resetQuick();
      packet.getData().add(heapArray);
      packet.setImageHeight(height);
      packet.setImageWidth(width);
      packet.setVideoSource(VideoSource.MULTISENSE_LEFT_EYE.toByte());
   }
}
