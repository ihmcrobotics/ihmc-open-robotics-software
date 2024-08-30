package us.ihmc.rdx.logging;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.*;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;
import us.ihmc.tools.string.StringTools;

import java.util.HashMap;
import java.util.function.Supplier;

import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFMPEGTools
{
   public static double rationalToFloatingPoint(AVRational rational)
   {
      return rational.num() / (double) rational.den();
   }

   public static void checkError(int returnCode, Pointer pointerToCheck, String message)
   {
      checkNonZeroError(returnCode, message);
      checkPointer(pointerToCheck, message);
   }

   public static void checkPointer(Pointer pointerToCheck, String message)
   {
      if (pointerToCheck == null)
         handleError(StringTools.format("pointer == null: {}", message), true);
      else if (pointerToCheck.isNull())
         handleError(StringTools.format("Pointer isNull() returned true: {}: {}", pointerToCheck.getClass().getSimpleName(), message), true);
   }

   public static void checkNegativeError(int returnCode, String message)
   {
      checkNegativeError(returnCode, message, true);
   }

   public static void checkNegativeError(int returnCode, String message, boolean throwException)
   {
      if (returnCode >= 0)
         return;

      handleError(StringTools.format("Code {} {}: {}", returnCode, getErrorCodeString(returnCode), message), throwException);
   }

   public static void checkNonZeroError(int returnCode, String message)
   {
      checkNonZeroError(returnCode, message, true);
   }

   public static void checkNonZeroError(int returnCode, String message, boolean throwException)
   {
      if (returnCode == 0)
         return;

      handleError(StringTools.format("Code {} {}: {}", returnCode, getErrorCodeString(returnCode), message), throwException);
   }

   private static void handleError(Supplier<String> messageSupplier, boolean throwException)
   {
      if (throwException)
      {
         LogTools.fatal(messageSupplier);
         throw new RuntimeException(messageSupplier.get());
      }
      else
      {
         LogTools.error(messageSupplier);
      }
   }

   public static String getErrorCodeString(int code)
   {
      return avutil.av_make_error_string(new BytePointer(1000), 1000, code).getString();
   }

   private static void mapAddNewValueOrAppend(HashMap<String, String> map, String key, String value)
   {
      if (map.containsKey(key))
         map.put(key, map.get(key) + ", " + value);
      else
         map.put(key, value);
   }

   public static int avPixelFormatToOpenCVType(int pixelFormat)
   {
      return switch (pixelFormat)
      {
         case AV_PIX_FMT_GRAY8 -> opencv_core.CV_8UC1;
         case AV_PIX_FMT_BGR24, AV_PIX_FMT_RGB24 -> opencv_core.CV_8UC3;
         case AV_PIX_FMT_RGBA, AV_PIX_FMT_BGRA -> opencv_core.CV_8UC4;
         case AV_PIX_FMT_GRAY16BE, AV_PIX_FMT_GRAY16LE -> opencv_core.CV_16UC1;
         default -> throw new NotImplementedException("Either the pixel format cannot be matched to an OpenCV type, or the match has not been implemented.");
      };
   }

   public static Mat avFrameToMat(AVFrame frame)
   {
      int openCVType = avPixelFormatToOpenCVType(frame.format());
      return new Mat(frame.height(), frame.width(), openCVType, frame.data());
   }

   public static void listLicenses()
   {
      HashMap<String, String> licenses = new HashMap<>();
      mapAddNewValueOrAppend(licenses, avcodec.avcodec_license().getString(), "avcodec");
      mapAddNewValueOrAppend(licenses, avdevice.avdevice_license().getString(), "avdevice");
      mapAddNewValueOrAppend(licenses, avfilter.avfilter_license().getString(), "avfilter");
      mapAddNewValueOrAppend(licenses, avformat.avformat_license().getString(), "avformat");
      mapAddNewValueOrAppend(licenses, avutil.avutil_license().getString(), "avutil");
      mapAddNewValueOrAppend(licenses, swresample.swresample_license().getString(), "swresample");
      mapAddNewValueOrAppend(licenses, swscale.swscale_license().getString(), "swscale");

      StringBuilder licensesStringBuilder = new StringBuilder();
      licensesStringBuilder.append("FFMPEG License(s):");
      licenses.forEach((String key, String value) ->
                       {
                          licensesStringBuilder.append(' ').append(key).append(": ").append(value).append(".");
                       });

      LogTools.debug(licensesStringBuilder.toString());
   }
}
