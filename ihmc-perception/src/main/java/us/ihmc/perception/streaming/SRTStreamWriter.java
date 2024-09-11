package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ffmpeg.FFMPEGTimeoutCallback;
import us.ihmc.perception.ffmpeg.FFMPEGTools;
import us.ihmc.perception.ffmpeg.FFMPEGVideoEncoder;

import java.net.InetSocketAddress;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

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

   private final FFMPEGTimeoutCallback timeoutCallback;

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

      timeoutCallback = new FFMPEGTimeoutCallback();

      ioContext = new AVIOContext();
      formatContext = new AVFormatContext();
      formatContext.interrupt_callback(timeoutCallback);
   }

   public boolean connect(double timeout)
   {
      // Open the IO
      timeoutCallback.start(timeout);
      error = avio_open2(ioContext, srtAddress, AVIO_FLAG_WRITE, timeoutCallback, this.ioOptions);
      if (error < 0)
         return false;

      // Create the output format context
      timeoutCallback.start(timeout);
      error = avformat_alloc_output_context2(formatContext, outputFormat, (String) null, null);
      if (!FFMPEGTools.checkError(error, formatContext, "Allocating output format context", false))
         return false;
      formatContext.pb(ioContext);

      // Get an output stream
      outputStream = encoder.newStream(formatContext);

      error = avformat_write_header(formatContext, formatOptions);
      if (!FFMPEGTools.checkNegativeError(error, "Sending header to caller", false))
         return false;

      LogTools.info("Connected to {}", srtAddress);

      connected = true;
      return true;
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
      if (connected)
         disconnect();

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

      timeoutCallback.close();

      if (outputStream != null)
         outputStream.close();
   }

   private void disconnect()
   {
      if (!connected)
         return;

      LogTools.info("Disconnecting from {}", srtAddress);

      error = av_write_trailer(formatContext);
      FFMPEGTools.checkNegativeError(error, "Writing trailer");

      connected = false;

      LogTools.info("Disconnected from {}", srtAddress);
   }
}
