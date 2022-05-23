package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.javacpp.BytePointer;

public class FFMPEGTools
{
   public static String getErrorCodeString(int code)
   {
      return avutil.av_make_error_string(new BytePointer(1000), 1000, code).getString();
   }
}
