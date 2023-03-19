package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacv.Java2DFrameUtils;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.ImageMessageFormat;

import java.awt.image.BufferedImage;
import java.time.Instant;

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

   public static void printMat(String name, Mat image)
   {
      LogTools.info(matToString(name, image));
   }

   public static void printMatVector(String name, MatVector matVector)
   {
      for (int i = 0; i < matVector.size(); i++)
      {
         LogTools.info(matToString("%s %d:".formatted(name, i), matVector.get(i)) + "\n");
      }
   }

   public static String matToString(String name, Mat image)
   {
      StringBuilder matString = new StringBuilder("Mat: [" + name + "]\n");

      for (int i = 0; i < image.rows(); i++)
      {
         for (int j = 0; j < image.cols(); j++)
         {
            if (image.type() == opencv_core.CV_16UC1)
               matString.append(image.ptr(i, j).getShort()).append("\t");
            if (image.type() == opencv_core.CV_64FC1)
               matString.append("%.5f\t".formatted(image.ptr(i, j).getDouble()));
            if (image.type() == opencv_core.CV_32FC2)
               matString.append("%.5f\t%.5f\t\t".formatted(image.ptr(i, j).getFloat(), image.ptr(i, j).getFloat(Float.BYTES)));
         }
         matString.append("\n");
      }

      return matString.toString();
   }

   public static void display(String tag, Mat image, int delay)
   {
      opencv_highgui.imshow(tag, image);
      int code = opencv_highgui.waitKeyEx(delay);
      if (code == 113) // Keycode for 'q'
      {
         System.exit(0);
      }
   }

   public static void displayDepth(String tag, Mat image, int delay)
   {
      displayDepth(tag, image, delay, 1.0f);
   }

   public static void displayDepth(String tag, Mat image, int delay, float scale)
   {
      Mat displayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC1);
      Mat finalDisplayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC3);

      displayDepth.convertTo(displayDepth, opencv_core.CV_8UC1, 0.8, 50);
      BytedecoOpenCVTools.clampTo8BitUnsignedChar(image, displayDepth, 0.0, 250.0);
      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

      opencv_imgproc.resize(finalDisplayDepth, finalDisplayDepth, new Size((int) (image.cols() * scale), (int) (image.rows() * scale)));
      display(tag, finalDisplayDepth, delay);
   }

   public static void displayHeightMap(String tag, Mat image, int delay, float scale)
   {
      Mat displayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC1);
      Mat finalDisplayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC3);

      BytedecoOpenCVTools.clampTo8BitUnsignedChar(image, displayDepth, 0.0, 250.0);

      opencv_imgproc.threshold(displayDepth, displayDepth, 100, 255, opencv_imgproc.CV_THRESH_TOZERO_INV);
      opencv_core.normalize(displayDepth, displayDepth, 255, 0, opencv_core.NORM_MINMAX, opencv_core.CV_8UC1, new Mat());

      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

      opencv_imgproc.resize(finalDisplayDepth, finalDisplayDepth, new Size((int) (image.cols() * scale), (int) (image.rows() * scale)));
      display(tag, finalDisplayDepth, delay);
   }

   public static void packVideoPacket(BytePointer compressedBytes, byte[] heapArray, VideoPacket packet, int height, int width, long nanoTime)
   {
      compressedBytes.asBuffer().get(heapArray, 0, compressedBytes.asBuffer().remaining());
      packet.setTimestamp(nanoTime);
      packet.getData().resetQuick();
      packet.getData().add(heapArray);
      packet.setImageHeight(height);
      packet.setImageWidth(width);
      packet.setVideoSource(VideoSource.MULTISENSE_LEFT_EYE.toByte());
   }

   public static void packImageMessage(ImageMessage imageMessage,
                                       BytePointer data,
                                       FramePose3D cameraPose,
                                       Instant aquisitionTime,
                                       long sequenceNumber,
                                       int height,
                                       int width,
                                       ImageMessageFormat format)
   {
      imageMessage.getData().resetQuick();
      for (int i = 0; i < data.limit(); i++)
      {
         imageMessage.getData().add(data.get(i));
      }
      format.packMessageFormat(imageMessage);
      imageMessage.setImageHeight(height);
      imageMessage.setImageWidth(width);
      imageMessage.getPosition().set(cameraPose.getPosition());
      imageMessage.getOrientation().set(cameraPose.getOrientation());
      imageMessage.setSequenceNumber(sequenceNumber);
      MessageTools.toMessage(aquisitionTime, imageMessage.getAcquisitionTime());
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

   /**
    * Puts 3 floats in a Mat that is an array of Float3s i.e. type == CV_32FC3
    * Assumes Mat is continuous. i.e. Mat::isContinuous == true
    */
   public static void putFloat3(BytePointer dataPointer, int float3Index, float float1, float float2, float float3)
   {
      dataPointer.putFloat(getFloat3ByteIndexContinuous(float3Index, 0), float1);
      dataPointer.putFloat(getFloat3ByteIndexContinuous(float3Index, 1), float2);
      dataPointer.putFloat(getFloat3ByteIndexContinuous(float3Index, 2), float3);
   }

   /**
    * Calculate the index of the first byte of float in a CV_32FC3 in a Mat that is an array of Float3s.
    * Assumes Mat is continuous. i.e. Mat::isContinuous == true
    * @param float3Index index of the float3 i.e. the triplet
    * @param floatIndex index of the float in the triplet, 0, 1, or 2
    */
   public static long getFloat3ByteIndexContinuous(int float3Index, int floatIndex)
   {
      return (long) float3Index * 3 * Float.BYTES + floatIndex * Float.BYTES;
   }
}
