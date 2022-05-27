package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.global.*;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import us.ihmc.log.LogTools;
import us.ihmc.tools.string.StringTools;

import java.util.function.Supplier;

public class FFMPEGTools
{
   public static void checkError(int returnCode, Pointer pointerToCheck, String message)
   {
      checkNonZeroError(returnCode, message);
      checkPointer(pointerToCheck, message);
   }

   public static void checkPointer(Pointer pointerToCheck, String message)
   {
      if (pointerToCheck == null)
      {
         Supplier<String> messageSupplier = StringTools.format("pointer == null: {}", message);
         LogTools.error(messageSupplier);
         throw new RuntimeException(messageSupplier.get());
      }
      else if (pointerToCheck.isNull())
      {
         Supplier<String> messageSupplier = StringTools.format("Pointer isNull() returned true: {}: {}", pointerToCheck.getClass().getSimpleName(), message);
         LogTools.error(messageSupplier);
         throw new RuntimeException(messageSupplier.get());
      }
   }

   public static void checkNonZeroError(int returnCode, String message)
   {
      if (returnCode != 0)
      {
         Supplier<String> messageSupplier = StringTools.format("Code {} {}: {}", returnCode, FFMPEGTools.getErrorCodeString(returnCode), message);
         LogTools.error(messageSupplier);
         throw new RuntimeException(messageSupplier.get());
      }
   }

   public static String getErrorCodeString(int code)
   {
      return avutil.av_make_error_string(new BytePointer(1000), 1000, code).getString();
   }

   public static void listLicenses() {
      LogTools.debug("FFMPEG Library Licenses:");
      LogTools.debug("avcodec:    " + avcodec.avcodec_license().getString());
      LogTools.debug("avdevice:   " + avdevice.avdevice_license().getString());
      LogTools.debug("avfilter:   " + avfilter.avfilter_license().getString());
      LogTools.debug("avformat:   " + avformat.avformat_license().getString());
      LogTools.debug("avutil:     " + avutil.avutil_license().getString());
      //LogTools.debug("postproc: " + postproc.postproc_license().getString()); //Unsatisfied link error
      LogTools.debug("swresample: " + swresample.swresample_license().getString());
      LogTools.debug("swscale:    " + swscale.swscale_license().getString());
   }
}
