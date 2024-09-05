package us.ihmc.perception.videoStreaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import us.ihmc.perception.ffmpeg.FFMPEGTools;
import us.ihmc.perception.ffmpeg.FFMPEGVideoEncoder;

import java.net.InetSocketAddress;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_copy;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class SRTStreamWriter
{
   private final FFMPEGVideoEncoder encoder;

   private final String srtAddress;
   private final AVOutputFormat outputFormat;

   private final AVIOContext ioContext;
   private final AVDictionary ioOptions;

   private final AVFormatContext formatContext;
   private final AVDictionary formatOptions;
   private final AVPacket packetCopy;
   private AVStream outputStream;

   private boolean initialized = false;
   private boolean connected = false;

   private int error;

   public SRTStreamWriter(FFMPEGVideoEncoder encoder,
                          InetSocketAddress outputAddress,
                          AVOutputFormat outputFormat,
                          AVDictionary ioOptions,
                          AVDictionary formatOptions)
   {
      this.encoder = encoder;
      this.outputFormat = outputFormat;
      srtAddress = StreamingTools.toSRTAddress(outputAddress);

      // Copy the IO options
      this.ioOptions = new AVDictionary();
      error = av_dict_copy(this.ioOptions, ioOptions, 0);
      FFMPEGTools.checkError(error, this.ioOptions, "Copying IO options");

      // Copy the format options
      this.formatOptions = new AVDictionary();
      error = av_dict_copy(this.formatOptions, formatOptions, 0);
      FFMPEGTools.checkError(error, this.ioOptions, "Copying format options");

      packetCopy = av_packet_alloc();
      FFMPEGTools.checkPointer(packetCopy, "Allocating a packet");

      ioContext = new AVIOContext();
      formatContext = new AVFormatContext();
   }

   public boolean connect()
   {
      // Open the IO
      error = avio_open2(ioContext, srtAddress, AVIO_FLAG_WRITE, null, this.ioOptions);
      if (!FFMPEGTools.checkError(error, ioContext, "Opening IO context", false))
         return false;

      // Create the output format context
      error = avformat_alloc_output_context2(formatContext, outputFormat, (String) null, null);
      if (!FFMPEGTools.checkError(error, formatContext, "Allocating output format context", false))
         return false;
      formatContext.pb(ioContext);

      // Get an output stream
      outputStream = encoder.newStream(formatContext);
      connected = true;
      return true;
   }

   public boolean write(AVPacket packetToWrite)
   {
      if (!initialized)
      {
         error = avformat_write_header(formatContext, formatOptions);
         if (!FFMPEGTools.checkNegativeError(error, "Sending header to caller", false))
            connected = false;

         initialized = true;
      }

      if (connected)
      {
         av_packet_ref(packetCopy, packetToWrite);

         av_packet_rescale_ts(packetCopy, encoder.getTimeBase(), outputStream.time_base());
         packetCopy.stream_index(outputStream.index());
         error = av_interleaved_write_frame(formatContext, packetCopy);
         if (error < 0)
            connected = false;

         av_packet_unref(packetCopy);
      }

      return connected;
   }

   public boolean isConnected()
   {
      return connected;
   }

   public void destroy()
   {
      connected = false;

      avio_closep(ioContext);
      ioContext.close();

      av_dict_free(ioOptions);
      ioOptions.close();

      av_packet_free(packetCopy);
      packetCopy.close();

      avformat_free_context(formatContext);
      formatContext.close();

      av_dict_free(formatOptions);
      formatOptions.close();

      if (outputStream != null)
         outputStream.close();
   }
}
