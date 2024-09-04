package us.ihmc.perception.videoStreaming;

import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;

import java.time.Instant;

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR24;

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
      double reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

      videoStreamer = new SRTVideoStreamer(reportedFPS, AV_PIX_FMT_BGR24);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "SRTStreamerDemoDestruction"));

      run();
   }

   private void run()
   {
      LogTools.info("Waiting for connection...");
      videoStreamer.getStreamOutputList().addOutput("127.0.0.1", 60001);

      LogTools.info("Got a connection! Streaming!");
      while (!shutdown && videoStreamer.getStreamOutputList().size() > 0)
      {
         videoCapture.read(frame);
         RawImage image = new RawImage(0, Instant.now(), 0.0f, frame, null, 0.0f, 0.0f, 0.0f, 0.0f, new FramePoint3D(), new FrameQuaternion());
         videoStreamer.setNextFrame(image);
         image.release();
      }

      shutdownReady.set();
   }

   private void destroy()
   {
      shutdown = true;

      shutdownReady.blockingPoll();

      videoCapture.close();
      videoStreamer.close();
      frame.close();
   }

   public static void main(String[] args)
   {
      new SRTVideoStreamerDemo();
   }
}
