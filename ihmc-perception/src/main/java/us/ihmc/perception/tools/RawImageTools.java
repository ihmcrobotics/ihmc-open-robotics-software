package us.ihmc.perception.tools;

import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;

public class RawImageTools
{
   private static final int DEFAULT_INTERPOLATION = opencv_imgproc.INTER_LINEAR;

   public static RawImage resize(RawImage imageToScale, int outputWidth, int outputHeight)
   {
      return resize(imageToScale, outputWidth, outputHeight, DEFAULT_INTERPOLATION);
   }

   public static RawImage resize(RawImage imageToResize, int outputWidth, int outputHeight, int openCVInterpolation)
   {
      if (imageToResize.get() == null)
         throw new RuntimeException("Input image was deallocated before it could be processed.");

      if (imageToResize.getWidth() == outputWidth && imageToResize.getHeight() == outputHeight)
         return imageToResize;

      Size outputSize = new Size(outputWidth, outputHeight);

      Mat resizedCpuMat = null;
      GpuMat resizedGpuMat = null;

      if (imageToResize.hasGpuImage())
      {
         GpuMat originalMat = imageToResize.getGpuImageMat();
         resizedGpuMat = new GpuMat();
         opencv_cudawarping.resize(originalMat, resizedGpuMat, outputSize, 0.0, 0.0, openCVInterpolation, null);
      }
      else
      {
         Mat originalMat = imageToResize.getCpuImageMat();
         resizedCpuMat = new Mat();
         opencv_imgproc.resize(originalMat, resizedCpuMat, outputSize, 0.0, 0.0, openCVInterpolation);
      }

      RawImage resizedImage = new RawImage(resizedCpuMat,
                                           resizedGpuMat,
                                           imageToResize.getPixelFormat(),
                                           resize(imageToResize.getIntrinsicsCopy(), outputWidth, outputHeight),
                                           imageToResize.getCameraModel(),
                                           imageToResize.getTransformToWorld(),
                                           imageToResize.getAcquisitionTime(),
                                           imageToResize.getSequenceNumber(),
                                           imageToResize.getDepthDiscretization());

      imageToResize.release();
      outputSize.close();

      return resizedImage;
   }

   public static CameraIntrinsics resize(CameraIntrinsics intrinsicsToResize, int outputWidth, int outputHeight)
   {
      if (intrinsicsToResize.getWidth() == outputWidth && intrinsicsToResize.getHeight() == outputHeight)
         return new CameraIntrinsics(intrinsicsToResize);

      double scaleX = (double) outputWidth / intrinsicsToResize.getWidth();
      double scaleY = (double) outputHeight / intrinsicsToResize.getHeight();

      return new CameraIntrinsics(outputHeight,
                                  outputWidth,
                                  scaleX * intrinsicsToResize.getFx(),
                                  scaleY * intrinsicsToResize.getFy(),
                                  scaleX * intrinsicsToResize.getCx(),
                                  scaleY * intrinsicsToResize.getCy());
   }

   public static RawImage scale(RawImage imageToScale, double scale)
   {
      return scale(imageToScale, scale, DEFAULT_INTERPOLATION);
   }

   public static RawImage scale(RawImage imageToScale, double scale, int openCVInterpolation)
   {
      return scale(imageToScale, scale, scale, openCVInterpolation);
   }

   public static RawImage scale(RawImage imageToScale, double scaleX, double scaleY)
   {
      return scale(imageToScale, scaleX, scaleY, DEFAULT_INTERPOLATION);
   }

   public static RawImage scale(RawImage imageToScale, double scaleX, double scaleY, int openCVInterpolation)
   {
      if (imageToScale.get() == null)
         throw new RuntimeException("Input image was deallocated before it could be processed.");

      int outputWidth = (int) Math.round(scaleX * imageToScale.getWidth());
      int outputHeight = (int) Math.round(scaleY * imageToScale.getHeight());

      RawImage scaledImage = resize(imageToScale, outputWidth, outputHeight, openCVInterpolation);

      imageToScale.release();

      return scaledImage;
   }

   public static CameraIntrinsics scale(CameraIntrinsics intrinsicsToScale, double scale)
   {
      return scale(intrinsicsToScale, scale, scale);
   }

   public static CameraIntrinsics scale(CameraIntrinsics intrinsicsToScale, double scaleX, double scaleY)
   {
      return new CameraIntrinsics((int) Math.round(scaleX * intrinsicsToScale.getHeight()),
                                  (int) Math.round(scaleY * intrinsicsToScale.getWidth()),
                                  scaleX * intrinsicsToScale.getFx(),
                                  scaleY * intrinsicsToScale.getFy(),
                                  scaleX * intrinsicsToScale.getCx(),
                                  scaleY * intrinsicsToScale.getCy());
   }

   public static RawImage convertColor(RawImage imageToConvert, PixelFormat newPixelFormat)
   {
      if (imageToConvert.get() == null)
         throw new RuntimeException("Input image was deallocated before it could be processed.");

      if (imageToConvert.getPixelFormat() == newPixelFormat)
         return imageToConvert;

      RawImage result;
      if (imageToConvert.hasGpuImage())
      {
         GpuMat newImage = new GpuMat();
         imageToConvert.getPixelFormat().convertToPixelFormat(imageToConvert.getGpuImageMat(), newImage, newPixelFormat);
         result = imageToConvert.replaceImage(newImage, newPixelFormat);
      }
      else
      {
         Mat newImage = new Mat();
         imageToConvert.getPixelFormat().convertToPixelFormat(imageToConvert.getCpuImageMat(), newImage, newPixelFormat);
         result = imageToConvert.replaceImage(newImage, newPixelFormat);
      }

      imageToConvert.release();

      return result;
   }
}
