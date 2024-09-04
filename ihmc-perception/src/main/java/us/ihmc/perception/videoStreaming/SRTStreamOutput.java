package us.ihmc.perception.videoStreaming;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import us.ihmc.perception.ffmpeg.FFMPEGTools;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_copy;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class SRTStreamOutput
{
   private final AVIOContext ioContext;
   private final AVDictionary ioOptions;

   private final AVFormatContext formatContext;
   private final AVDictionary formatOptions;

   private boolean initialized = false;
   private boolean connected = false;

   private int error;

   public SRTStreamOutput(String outputAddress, int outputPort, String outputFormat, AVDictionary ioOptions, AVDictionary formatOptions)
   {
      String formattedAddress = "srt://" + outputAddress + ":" + outputPort;

      // Copy the IO options
      this.ioOptions = new AVDictionary();
      error = av_dict_copy(this.ioOptions, ioOptions, 0);
      FFMPEGTools.checkError(error, this.ioOptions, "Copying IO options");

      // Copy the format options
      this.formatOptions = new AVDictionary();
      error = av_dict_copy(this.formatOptions, formatOptions, 0);
      FFMPEGTools.checkError(error, this.ioOptions, "Copying format options");

      // Open the IO
      ioContext = new AVIOContext();
      error = avio_open2(ioContext, formattedAddress, AVIO_FLAG_WRITE, null, this.ioOptions);
      FFMPEGTools.checkError(error, ioContext, "Opening IO context");

      // Create the output format context
      formatContext = new AVFormatContext();
      error = avformat_alloc_output_context2(formatContext, null, outputFormat, null);
      FFMPEGTools.checkError(error, formatContext, "Allocating output format context");
      formatContext.pb(ioContext);
   }

   public boolean write(AVPacket packetToWrite)
   {
      if (!initialized)
      {
         error = avformat_write_header(formatContext, formatOptions);
         if (error < 0)
            return false;

         connected = true;
         initialized = true;
      }

      if (connected)
      {
         error = av_interleaved_write_frame(formatContext, packetToWrite);
         if (error < 0)
            connected = false;
      }

      return connected;
   }

   public boolean isConntected()
   {
      return connected;
   }

   public AVFormatContext getFormatContext()
   {
      return formatContext;
   }

   public AVIOContext getIOContext()
   {
      return ioContext;
   }

   public void close()
   {
      avio_closep(ioContext);
      ioContext.close();

      av_dict_free(ioOptions);
      ioOptions.close();

      avformat_free_context(formatContext);
      formatContext.close();

      av_dict_free(formatOptions);
      formatOptions.close();
   }
}
