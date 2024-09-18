package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ffmpeg.FFMPEGTools;
import us.ihmc.perception.ffmpeg.FFMPEGVideoEncoder;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_copy;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class SRTStreamWriter
{
   private final FFMPEGVideoEncoder encoder;

   private final AVIOContext srtContext;

   private final AVOutputFormat outputFormat;
   private final AVFormatContext formatContext;
   private final AVDictionary formatOptions;
   private final AVPacket packetCopy;
   private AVStream outputStream;

   private boolean connected = false;

   private int error;

   public SRTStreamWriter(FFMPEGVideoEncoder encoder, AVIOContext srtContext, AVOutputFormat outputFormat, AVDictionary formatOptions)
   {
      this.encoder = encoder;
      this.srtContext = srtContext;
      this.outputFormat = outputFormat;

      // Copy the format options
      this.formatOptions = new AVDictionary();
      error = av_dict_copy(this.formatOptions, formatOptions, 0);
      FFMPEGTools.checkNegativeError(error, "Copying format options");

      formatContext = new AVFormatContext();

      packetCopy = av_packet_alloc();
      FFMPEGTools.checkPointer(packetCopy, "Allocating a packet");
   }

   public boolean startOutput()
   {
      // Create the output format context
      LogTools.debug("Allocating output context");
      error = avformat_alloc_output_context2(formatContext, outputFormat, (String) null, null);
      if (!FFMPEGTools.checkError(error, formatContext, "Allocating output format context"))
         return false;
      formatContext.pb(srtContext);

      // Get an output stream from the encoder
      LogTools.debug("Got a new stream from encoder");
      outputStream = encoder.newStream(formatContext);

      // Write a header to the caller
      LogTools.debug("Writing header to caller");
      error = avformat_write_header(formatContext, formatOptions);
      if (!FFMPEGTools.checkNegativeError(error, "Sending header to caller", false))
         return false;

      FFMPEGTools.checkDictionaryAfterUse(formatOptions);

      LogTools.debug("Successfully started connection with caller");
      return connected = true;
   }

   public boolean write(AVPacket packetToWrite)
   {
      if (connected)
      {
         av_packet_ref(packetCopy, packetToWrite);

         av_packet_rescale_ts(packetCopy, encoder.getTimeBase(), outputStream.time_base());
         packetCopy.stream_index(outputStream.index());
         error = av_interleaved_write_frame(formatContext, packetCopy);
         if (error < 0)
         {
            connected = false;
            LogTools.debug("Connection failed with caller while writing packet");
         }

         av_packet_unref(packetCopy);
      }

      return connected;
   }

   public void endOutput()
   {
      if (!connected)
         return;

      LogTools.debug("Writing trailer to caller");
      error = av_write_trailer(formatContext);
      FFMPEGTools.checkNegativeError(error, "Writing trailer");
      LogTools.debug("Successfully ended connection with caller");

      connected = false;
   }

   public void destroy()
   {
      endOutput();

      avio_closep(srtContext);
      srtContext.close();

      av_packet_free(packetCopy);
      packetCopy.close();

      avformat_free_context(formatContext);
      formatContext.close();

      av_dict_free(formatOptions);
      formatOptions.close();

      if (outputStream != null)
         outputStream.close();
   }

   public boolean isConnected()
   {
      return connected;
   }
}
