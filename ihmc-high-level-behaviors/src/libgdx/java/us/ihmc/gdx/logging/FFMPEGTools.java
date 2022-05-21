package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.global.avutil;

import java.nio.ByteBuffer;

public class FFMPEGTools
{
   public static String getErrorCodeString(int code)
   {
      ByteBuffer errbuf = ByteBuffer.allocate(1000); //this is completely arbitrary
      avutil.av_make_error_string(errbuf, errbuf.capacity(), code);
      StringBuilder s = new StringBuilder();
      for (int i = 0; i < errbuf.capacity(); i++) {
         if (errbuf.get(i) == 0)
            break;

         s.append((char) errbuf.get(i));
      }

      return s.toString();
   }
}
