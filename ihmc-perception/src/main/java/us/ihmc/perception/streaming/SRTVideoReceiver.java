package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.ffmpeg.FFmpegTimeoutCallback;
import us.ihmc.perception.ffmpeg.FFmpegTools;
import us.ihmc.perception.ffmpeg.FFmpegVideoDecoder;

import java.net.InetSocketAddress;
import java.util.Map;

import static org.bytedeco.ffmpeg.global.avcodec.AV_PKT_FLAG_CORRUPT;
import static org.bytedeco.ffmpeg.global.avcodec.AV_PKT_FLAG_KEY;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.AVERROR_EOF;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class SRTVideoReceiver
{
   private FFmpegVideoDecoder decoder;
   private final int outputPixelFormat;

   private final AVDictionary srtOptions;
   private final AVFormatContext inputFormatContext;
   private final FFmpegTimeoutCallback timeoutCallback;

   private boolean connected = false;
   private boolean firstRead = false;
   private double readFrameTimeout = 1.0;
   private int packetFlagsToMatch = 0;
   private int packetFlagsToIgnore = 0;

   int error;

   public SRTVideoReceiver(int outputAVPixelFormat)
   {
      outputPixelFormat = outputAVPixelFormat;

      timeoutCallback = new FFmpegTimeoutCallback();

      srtOptions = new AVDictionary();

      inputFormatContext = avformat_alloc_context();
      FFmpegTools.checkPointer(inputFormatContext, "Allocating input format context");
      inputFormatContext.interrupt_callback(timeoutCallback);
   }

   public boolean connect(InetSocketAddress streamerAddress, double timeout)
   {
      return connect(StreamingTools.toSRTAddress(streamerAddress), timeout);
   }

   public boolean connect(String streamerSRTAddress, double timeout)
   {
      long timeoutMicroseconds = Conversions.nanosecondsToMicroseconds(Conversions.secondsToNanoseconds(timeout));

      // Set the SRT options
      Map<String, String> srtOptionMap = StreamingTools.getLiveSRTOptions();
      srtOptionMap.put("mode", "caller");
      srtOptionMap.put("timeout", String.valueOf(timeoutMicroseconds));
      FFmpegTools.setAVDictionary(srtOptions, srtOptionMap);

      // Connect to streamer
      error = avformat_open_input(inputFormatContext, streamerSRTAddress, null, srtOptions);
      if (error < 0)
         return false;

      FFmpegTools.checkDictionaryAfterUse(srtOptions);

      // Receive a few packets to get stream info
      timeoutCallback.start(timeout);
      error = avformat_find_stream_info(inputFormatContext, (AVDictionary) null);
      timeoutCallback.stop();
      if (!FFmpegTools.checkNegativeError(error, "Finding stream info on " + streamerSRTAddress, false))
         return false;

      // Create and initialize the decoder for the stream
      decoder = new FFmpegVideoDecoder(inputFormatContext, outputPixelFormat);
      decoder.initialize(null, this::getNextPacket);

      connected = true;
      firstRead = true;

      return true;
   }

   public void disconnect()
   {
      if (!connected)
         return;

      connected = false;

      avformat_close_input(inputFormatContext);

      if (decoder != null)
      {
         decoder.destroy();
         decoder = null;
      }
   }

   private int getNextPacket(AVPacket packetToPack)
   {
      long startTime = System.nanoTime();

      boolean packetRead;
      do
      {
         // Try reading a packet from the stream
         timeoutCallback.start(readFrameTimeout);
         error = av_read_frame(inputFormatContext, packetToPack);
         timeoutCallback.stop();

         // Ensure we read a packet, and if we did also ensure the flags match
         packetRead = error >= 0;
         if (packetRead && packetFlagsToMatch != 0)
         {
            packetRead = (packetToPack.flags() & packetFlagsToMatch) >= packetFlagsToMatch;
            packetRead &= (packetToPack.flags() & packetFlagsToIgnore) == 0;
         }

         if (!packetRead)
         {
            // Calculate time spent
            long currentTime = System.nanoTime();
            double timeTaken = Conversions.nanosecondsToSeconds(currentTime - startTime);

            // Waited too long. Stop trying
            if (timeTaken > readFrameTimeout && readFrameTimeout >= 0)
               break;

            // Wait a bit and retry
            ThreadTools.sleep(ThreadTools.REASONABLE_WAITING_SLEEP_DURATION_MS);
         }
      } while (!packetRead);

      // Couldn't receive any packets. Disconnect.
      if (!packetRead)
      {
         disconnect();
         return AVERROR_EOF();
      }

      firstRead = false;

      // Successfully received a packet
      return 0;
   }

   public Mat getNextFrame(double timeout)
   {
      if (!connected)
         return null;

      readFrameTimeout = timeout;
      packetFlagsToIgnore = AV_PKT_FLAG_CORRUPT;
      packetFlagsToMatch = 0;
      if (firstRead)
         packetFlagsToMatch = AV_PKT_FLAG_KEY;

      return decoder.getNextFrame();
   }

   public boolean isConnected()
   {
      return connected;
   }

   public void destroy()
   {
      if (isConnected())
         disconnect();

      av_dict_free(srtOptions);
      avformat_free_context(inputFormatContext);

      inputFormatContext.close();
      timeoutCallback.close();
   }
}
