package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.ffmpeg.FFMPEGTimeoutCallback;
import us.ihmc.perception.ffmpeg.FFMPEGTools;
import us.ihmc.perception.ffmpeg.FFMPEGVideoDecoder;

import java.net.InetSocketAddress;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.AVERROR_EOF;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class SRTVideoReceiver
{
   private FFMPEGVideoDecoder decoder;
   private final int outputPixelFormat;

   private final AVDictionary srtOptions;
   private final AVFormatContext inputFormatContext;
   private final FFMPEGTimeoutCallback timeoutCallback;

   private volatile boolean connected = false;
   private double readFrameTimeout = 1.0;

   int error;

   public SRTVideoReceiver(int outputAVPixelFormat)
   {
      outputPixelFormat = outputAVPixelFormat;

      timeoutCallback = new FFMPEGTimeoutCallback();

      srtOptions = new AVDictionary();

      inputFormatContext = avformat_alloc_context();
      FFMPEGTools.checkPointer(inputFormatContext, "Allocating input format context");
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
      FFMPEGTools.setAVDictionary(srtOptions, srtOptionMap);

      // Connect to streamer
      error = avformat_open_input(inputFormatContext, streamerSRTAddress, null, srtOptions);
      if (error < 0)
         return false;

      FFMPEGTools.checkDictionaryAfterUse(srtOptions);

      // Receive a few packets to get stream info
      timeoutCallback.start(timeout);
      error = avformat_find_stream_info(inputFormatContext, (AVDictionary) null);
      timeoutCallback.stop();
      if (!FFMPEGTools.checkNegativeError(error, "Finding stream info on " + streamerSRTAddress, false))
         return false;

      // Create and initialize the decoder for the stream
      decoder = new FFMPEGVideoDecoder(inputFormatContext, outputPixelFormat);
      decoder.initialize(null, this::getNextPacket);

      connected = true;

      return true;
   }

   public void disconnect()
   {
      if (!connected)
         return;

      connected = false;

      avformat_close_input(inputFormatContext);
      decoder.destroy();
      decoder = null;
   }

   private int getNextPacket(AVPacket packetToPack)
   {
      long startTime = System.nanoTime();
      do
      {
         // Try reading a packet from the stream
         timeoutCallback.start(ThreadTools.REASONABLE_WAITING_SLEEP_DURATION_MS, TimeUnit.MILLISECONDS);
         error = av_read_frame(inputFormatContext, packetToPack);
         timeoutCallback.stop();

         if (error < 0) // Failed to read packet
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
      } while (error < 0);

      // Couldn't receive any packets. Disconnect.
      if (error < 0)
      {
         disconnect();
         return AVERROR_EOF();
      }

      // Successfully received a packet
      return 0;
   }

   public Mat getNextFrame(double timeout)
   {
      if (!connected)
         return null;

      readFrameTimeout = timeout;
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
