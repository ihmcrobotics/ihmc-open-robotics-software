package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.global.avutil;

public class FFMPEGTools
{
   public static String getErrorCodeString(int code)
   {
      if (code == avutil.AVERROR_BUG)
         return "AVERROR_BUG";
      if (code == avutil.AVERROR_BUFFER_TOO_SMALL)
         return "AVERROR_BUFFER_TOO_SMALL";
      if (code == avutil.AVERROR_DECODER_NOT_FOUND)
         return "AVERROR_DECODER_NOT_FOUND";
      if (code == avutil.AVERROR_DEMUXER_NOT_FOUND)
         return "AVERROR_DEMUXER_NOT_FOUND";
      if (code == avutil.AVERROR_ENCODER_NOT_FOUND)
         return "AVERROR_ENCODER_NOT_FOUND";
      if (code == avutil.AVERROR_EOF)
         return "AVERROR_EOF";
      if (code == avutil.AVERROR_EXIT)
         return "AVERROR_EXIT";
      if (code == avutil.AVERROR_EXTERNAL)
         return "AVERROR_EXTERNAL";
      if (code == avutil.AVERROR_FILTER_NOT_FOUND)
         return "AVERROR_FILTER_NOT_FOUND";
      if (code == avutil.AVERROR_INVALIDDATA)
         return "AVERROR_INVALIDDATA";
      if (code == avutil.AVERROR_MUXER_NOT_FOUND)
         return "AVERROR_MUXER_NOT_FOUND";
      if (code == avutil.AVERROR_OPTION_NOT_FOUND)
         return "AVERROR_OPTION_NOT_FOUND";
      if (code == avutil.AVERROR_PATCHWELCOME)
         return "AVERROR_PATCHWELCOME";
      if (code == avutil.AVERROR_PROTOCOL_NOT_FOUND)
         return "AVERROR_PROTOCOL_NOT_FOUND";
      if (code == avutil.AVERROR_STREAM_NOT_FOUND)
         return "AVERROR_STREAM_NOT_FOUND";
      if (code == avutil.AVERROR_BUG2)
         return "AVERROR_BUG2";
      if (code == avutil.AVERROR_UNKNOWN)
         return "AVERROR_UNKNOWN";
      if (code == avutil.AVERROR_EXPERIMENTAL)
         return "AVERROR_EXPERIMENTAL";
      if (code == avutil.AVERROR_INPUT_CHANGED)
         return "AVERROR_INPUT_CHANGED";
      if (code == avutil.AVERROR_OUTPUT_CHANGED)
         return "AVERROR_OUTPUT_CHANGED";
      if (code == avutil.AVERROR_HTTP_BAD_REQUEST)
         return "AVERROR_HTTP_BAD_REQUEST";
      if (code == avutil.AVERROR_HTTP_UNAUTHORIZED)
         return "AVERROR_HTTP_UNAUTHORIZED";
      if (code == avutil.AVERROR_HTTP_FORBIDDEN)
         return "AVERROR_HTTP_FORBIDDEN";
      if (code == avutil.AVERROR_HTTP_NOT_FOUND)
         return "AVERROR_HTTP_NOT_FOUND";
      if (code == avutil.AVERROR_HTTP_OTHER_4XX)
         return "AVERROR_HTTP_OTHER_4XX";
      if (code == avutil.AVERROR_HTTP_SERVER_ERROR)
         return "AVERROR_HTTP_SERVER_ERROR";

      return "Code not found.";
   }
}
