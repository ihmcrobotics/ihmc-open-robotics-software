package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVDictionaryEntry;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.avutil.AVPixFmtDescriptor;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avdevice;
import org.bytedeco.ffmpeg.global.avfilter;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.ffmpeg.global.swresample;
import org.bytedeco.ffmpeg.global.swscale;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import us.ihmc.log.LogTools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.tools.string.StringTools;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFmpegTools
{
   public static double rationalToFloatingPoint(AVRational rational)
   {
      return rational.num() / (double) rational.den();
   }

   public static Mat avFrameToMat(AVFrame frame)
   {
      Mat resultMat;
      AVPixFmtDescriptor pixelFormatDescriptor = av_pix_fmt_desc_get(frame.format());
      if ((pixelFormatDescriptor.flags() & AV_PIX_FMT_FLAG_PLANAR) == 0)
      {  // Non-planar data; create Mat directly using 1st plane
         int openCVType = PixelFormat.fromFFmpegPixelFormat(frame.format()).toOpenCVType();
         resultMat = new Mat(frame.height(), frame.width(), openCVType, frame.data(0), frame.linesize(0));
      }
      else
      {  // Planar data; must extract planes individually then merge into 1 Mat
         MatVector planes = new MatVector();
         int openCVType = PixelFormat.fromFFmpegPixelFormat(frame.format()).toOpenCVType(1);
         for (int i = 0; frame.data(i) != null && !frame.data(i).isNull(); ++i)
            planes.push_back(new Mat(frame.height(), frame.width(), openCVType, frame.data(i), frame.linesize(i)));

         resultMat = new Mat();
         opencv_core.merge(planes, resultMat);
         planes.close();
      }
      pixelFormatDescriptor.close();

      return resultMat;
   }

   public static void setAVDictionary(AVDictionary dictionaryToSet, Map<String, String> options)
   {
      for (Map.Entry<String, String> option : options.entrySet())
         av_dict_set(dictionaryToSet, option.getKey(), option.getValue(), 0);
   }

   public static void checkDictionaryAfterUse(AVDictionary dictionary)
   {
      if (av_dict_count(dictionary) == 0)
         return;

      StringBuilder notFoundEntriesString = new StringBuilder("The following entries did not match available options:\n");
      AVDictionaryEntry entry = null;
      while ((entry = av_dict_iterate(dictionary, entry)) != null)
         notFoundEntriesString.append("\t").append(entry.key().getString()).append(":").append(entry.value().getString()).append("\n");

      LogTools.warn(notFoundEntriesString);
   }

   public static boolean checkError(int returnCode, Pointer pointerToCheck, String message)
   {
      return checkError(returnCode, pointerToCheck, message, true);
   }

   public static boolean checkError(int returnCode, Pointer pointerToCheck, String message, boolean throwException)
   {
      return checkNonZeroError(returnCode, message, throwException) && checkPointer(pointerToCheck, message, throwException);
   }

   public static boolean checkPointer(Pointer pointerToCheck, String message)
   {
      return checkPointer(pointerToCheck, message, true);
   }

   public static boolean checkPointer(Pointer pointerToCheck, String message, boolean throwException)
   {
      if (pointerToCheck == null)
      {
         handleError(StringTools.format("pointer == null: {}", message), throwException);
         return false;
      }
      else if (pointerToCheck.isNull())
      {
         handleError(StringTools.format("Pointer isNull() returned true: {}: {}", pointerToCheck.getClass().getSimpleName(), message), throwException);
         return false;
      }

      return true;
   }

   public static boolean checkNegativeError(int returnCode, String message)
   {
      return checkNegativeError(returnCode, message, true);
   }

   public static boolean checkNegativeError(int returnCode, String message, boolean throwException)
   {
      if (returnCode >= 0)
         return true;

      handleError(StringTools.format("Code {} {}: {}", returnCode, getErrorCodeString(returnCode), message), throwException);
      return false;
   }

   public static boolean checkNonZeroError(int returnCode, String message)
   {
      return checkNonZeroError(returnCode, message, true);
   }

   public static boolean checkNonZeroError(int returnCode, String message, boolean throwException)
   {
      if (returnCode == 0)
         return true;

      handleError(StringTools.format("Code {} {}: {}", returnCode, getErrorCodeString(returnCode), message), throwException);
      return false;
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
