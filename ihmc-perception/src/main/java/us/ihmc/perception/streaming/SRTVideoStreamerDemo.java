package us.ihmc.perception.streaming;

import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.thread.Notification;
import us.ihmc.log.LogTools;

import java.net.InetSocketAddress;
import java.time.Instant;

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR24;

/**
 * <p>
 * Demo for the {@link SRTVideoStreamer}. Grabs images from a webcam
 * and gives them to the streamer to be streamed over SRT.
 * </p>
 * <p>
 * To view the stream, use the following command:
 * {@code ffplay srt://[streamer-address:streamer-port] -fflags nobuffer}.
 * Alternatively, you can run the {@link SRTVideoReceiverDemo},
 * but be sure to set the {@link SRTVideoReceiverDemo#STREAMER_ADDRESS} correctly.
 * </p>
 */
public class SRTVideoStreamerDemo
{
   private final VideoCapture videoCapture;
   private final Mat frame;
   private final SRTVideoStreamer videoStreamer;

   private boolean shutdown = false;
   private final Notification shutdownReady = new Notification();

   private SRTVideoStreamerDemo()
   {
      // Open a webcam
      videoCapture = new VideoCapture(-1);

      frame = new Mat();

      // Read image data from webcam
      int imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);

      // Create and initialize the video streamer
      InetSocketAddress streamerAddress = StreamingTools.getHostAddress();
      LogTools.info("Starting streamer on {}", streamerAddress);
      videoStreamer = new SRTVideoStreamer(streamerAddress);
      videoStreamer.initialize(imageWidth, imageHeight, AV_PIX_FMT_BGR24);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "SRTStreamerDemoDestruction"));

      run();
   }

   private void run()
   {
      while (!shutdown)
      {
         videoCapture.read(frame);
         videoStreamer.sendFrame(frame, Instant.now());
      }

      shutdownReady.set();
   }

   private void destroy()
   {
      shutdown = true;

      shutdownReady.blockingPoll();

      videoCapture.close();
      videoStreamer.destroy();
      frame.close();
   }

   public static void main(String[] args)
   {
      new SRTVideoStreamerDemo();
   }
}
