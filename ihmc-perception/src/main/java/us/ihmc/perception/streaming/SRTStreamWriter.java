package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ffmpeg.FFmpegTools;
import us.ihmc.perception.ffmpeg.FFmpegVideoEncoder;

import java.util.Timer;
import java.util.TimerTask;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_copy;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class SRTStreamWriter
{
   private static final long CONNECTION_TIMEOUT_MS = 2000L;

   private final FFmpegVideoEncoder encoder;

   private final AVIOContext srtContext;

   private final AVOutputFormat outputFormat;
   private final AVFormatContext formatContext;
   private final AVDictionary formatOptions;
   private final AVPacket packetCopy;
   private AVStream outputStream;

   private final Timer connectionTimeoutTimer;
   private TimerTask endConnectionOnTimeout;
   private boolean connected = false;

   private long lastPacketPTS = Long.MIN_VALUE;
   private long lastPacketDTS = Long.MIN_VALUE;
   private boolean allowNonIncreasingTimestamp;

   private int error;

   public SRTStreamWriter(FFmpegVideoEncoder encoder, AVIOContext srtContext, AVOutputFormat outputFormat, AVDictionary formatOptions)
   {
      this.encoder = encoder;
      this.srtContext = srtContext;
      this.outputFormat = outputFormat;

      // Copy the format options
      this.formatOptions = new AVDictionary();
      error = av_dict_copy(this.formatOptions, formatOptions, 0);
      FFmpegTools.checkNegativeError(error, "Copying format options");

      formatContext = new AVFormatContext();

      packetCopy = av_packet_alloc();
      FFmpegTools.checkPointer(packetCopy, "Allocating a packet");

      connectionTimeoutTimer = new Timer(getClass().getSimpleName() + "ConnectionTimeout", true);

      allowNonIncreasingTimestamp = false;
   }

   public void allowNonIncreasingTimestamp(boolean allow)
   {
      allowNonIncreasingTimestamp = allow;
   }

   private boolean startOutput()
   {
      // Create the output format context
      LogTools.debug("Allocating output context");
      error = avformat_alloc_output_context2(formatContext, outputFormat, (String) null, null);
      if (!FFmpegTools.checkError(error, formatContext, "Allocating output format context"))
         return false;
      formatContext.pb(srtContext);

      // Get an output stream from the encoder
      LogTools.debug("Got a new stream from encoder");
      outputStream = encoder.newStream(formatContext);

      // Write a header to the caller
      LogTools.debug("Writing header to caller");
      error = avformat_write_header(formatContext, formatOptions);
      if (!FFmpegTools.checkNegativeError(error, "Sending header to caller", false))
         return false;

      FFmpegTools.checkDictionaryAfterUse(formatOptions);

      LogTools.debug("Successfully started connection with caller");
      return true;
   }

   public synchronized boolean write(AVPacket packetToWrite)
   {
      // Check timestamps (pts, dts) to make sure they're increasing
      long pts = packetToWrite.pts();
      long dts = packetToWrite.dts();
      if ((pts <= lastPacketPTS || dts <= lastPacketDTS) && !allowNonIncreasingTimestamp)
      {
         LogTools.debug("PTS (%d) <= last PTS (%d) || DTS (%d) <= last DTS (%d)".formatted(pts, lastPacketPTS, dts, lastPacketDTS));
         return connected;
      }
      lastPacketPTS = pts;
      lastPacketDTS = dts;

      // If we haven't started, start
      if (outputStream == null)
         connected = startOutput();

      // Check that connection hasn't timed out
      if (endConnectionOnTimeout != null)
         connected &= endConnectionOnTimeout.cancel();

      // If we're still connected, try sending the packet
      if (connected)
      {
         av_packet_ref(packetCopy, packetToWrite);

         av_packet_rescale_ts(packetCopy, encoder.getTimeBase(), outputStream.time_base());
         packetCopy.stream_index(outputStream.index());
         error = av_interleaved_write_frame(formatContext, packetCopy);
         if (error < 0)
         {
            connected = false;
            LogTools.debug("Connection failed with caller while writing packet ({})", FFmpegTools.getErrorCodeString(error));
         }

         endConnectionOnTimeout = new ConnectionTimeoutTask();
         connectionTimeoutTimer.schedule(endConnectionOnTimeout, CONNECTION_TIMEOUT_MS);
         av_packet_unref(packetCopy);
      }

      return connected;
   }

   public synchronized void endOutput()
   {
      if (!connected)
         return;

      LogTools.debug("Writing trailer to caller");
      error = av_write_trailer(formatContext);
      FFmpegTools.checkNegativeError(error, "Writing trailer");
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

   public synchronized boolean isConnected()
   {
      return connected;
   }

   private class ConnectionTimeoutTask extends TimerTask
   {
      @Override
      public void run()
      {
         endOutput();
      }
   }
}
