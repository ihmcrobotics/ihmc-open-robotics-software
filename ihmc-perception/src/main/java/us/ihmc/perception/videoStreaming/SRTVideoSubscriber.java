package us.ihmc.perception.videoStreaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.ffmpeg.FFMPEGTools;
import us.ihmc.perception.ffmpeg.FFMPEGVideoDecoder;

import java.net.InetSocketAddress;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.AVERROR_EOF;

// TODO: Make abstract SRTSubscriber class and extend to video and audio
public class SRTVideoSubscriber
{
   private static final int MAX_FAILED_READS = 64;
   private static final int RETRY_SLEEP_DURATION_MS = 8;

   private final String srtInputAddress;

   private FFMPEGVideoDecoder decoder;
   private final int outputPixelFormat;

   private final AVFormatContext inputFormatContext;

   private boolean connected = false;

   int error;

   public SRTVideoSubscriber(InetSocketAddress inputAddress, int outputAVPixelFormat)
   {
      outputPixelFormat = outputAVPixelFormat;

      srtInputAddress = StreamingTools.toSRTAddress(inputAddress);
      inputFormatContext = avformat_alloc_context();
      FFMPEGTools.checkPointer(inputFormatContext, "Allocating input format context");
   }

   public boolean connect()
   {
      error = avformat_open_input(inputFormatContext, srtInputAddress, null, null);
      if (error < 0)
         return false;

      error = avformat_find_stream_info(inputFormatContext, (AVDictionary) null);
      if (!FFMPEGTools.checkNegativeError(error, "Finding stream info on " + srtInputAddress))
         return false;

      decoder = new FFMPEGVideoDecoder(inputFormatContext, outputPixelFormat);
      decoder.initialize(null, this::getNextPacket);

      connected = true;

      return true;
   }

   public boolean waitForConnection(double timeoutSeconds)
   {
      long startTime = System.nanoTime();
      do
      {
         // Try to connect
         connect();

         // Check if connection failed
         if (!connected)
         {
            // Calculate time spent
            long currentTime = System.nanoTime();
            double timeTaken = Conversions.nanosecondsToSeconds(currentTime - startTime);

            // Waited too long. Stop trying
            if (timeTaken > timeoutSeconds && timeoutSeconds >= 0)
               break;

            // Wait a little and retry
            ThreadTools.sleep(RETRY_SLEEP_DURATION_MS);
         }
      } while (!connected);

      return connected;
   }

   public void disconnect()
   {
      if (!connected)
         return;

      avformat_close_input(inputFormatContext);
      decoder.destroy();
      decoder = null;

      connected = false;
   }

   private int getNextPacket(AVPacket packetToPack)
   {
      int failedReads = 0;
      do
      {
         // Try reading a packet from the stream
         error = av_read_frame(inputFormatContext, packetToPack);

         if (error < 0)
         {
            // Failed to read packet; wait a bit and retry
            ++failedReads;
            ThreadTools.sleep(RETRY_SLEEP_DURATION_MS);
         }
      } while (error < 0 && failedReads < MAX_FAILED_READS);

      // Couldn't receive any packets. Disconnect.
      if (error < 0)
      {
         disconnect();
         return AVERROR_EOF();
      }

      // Successfully received a packet
      return 0;
   }

   public Mat getNextImage()
   {
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

      inputFormatContext.close();
   }
}
