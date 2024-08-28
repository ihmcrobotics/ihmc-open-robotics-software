package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.thread.Notification;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

/**
 * FFMPEG demo which reads images from a webcam and streams an H.264 encoded video over UDP.
 * To view the video, use the following command:
 * {@code ffplay udp://127.0.0.1:60001 -fflags nobuffer}
 */
public class FFMPEGWebcamUDPStreamingDemo
{
   private final VideoCapture videoCapture = new VideoCapture(-1);

   private final AVFormatContext outputContext = new AVFormatContext();
   private final AVIOContext ioContext = new AVIOContext();
   private final FFMPEGVideoEncoder videoEncoder;

   private boolean done = false;
   private boolean shutdown = false;
   private final Notification readyForShutdown = new Notification();

   private FFMPEGWebcamUDPStreamingDemo()
   {
      // Read image data from webcam
      int imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
      double reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

      // Open UDP IO
      int error = avio_open(ioContext, "udp://127.0.0.1:60001", AVIO_FLAG_WRITE);
      FFMPEGTools.checkError(error, ioContext, "Openning UDP port");

      // Create an output context
      error = avformat_alloc_output_context2(outputContext, null, "h264", null);
      FFMPEGTools.checkError(error, outputContext, "Allocating UDP Output Context");
      outputContext.pb(ioContext);
      outputContext.flush_packets(1);

      // Initialize video encoder
      videoEncoder = new FFMPEGVideoEncoder(outputContext,
                                            "h264_nvenc",
                                            400000,
                                            imageWidth,
                                            imageHeight,
                                            AV_PIX_FMT_YUV420P,
                                            reportedFPS,
                                            4,
                                            0,
                                            AV_PIX_FMT_BGR24);

      // Set flags for low latency
      AVDictionary encoderFlags = new AVDictionary();
      av_dict_set(encoderFlags, "preset", "p1", 0);
      av_dict_set(encoderFlags, "tune", "ull", 0);
      av_dict_set(encoderFlags, "profile", "high", 0);
      av_dict_set(encoderFlags, "zerolatency", "1", 0);
      av_dict_set(encoderFlags, "maxrate", "500000", 0);
      av_dict_set(encoderFlags, "bufsize", "1000000", 0);
      av_dict_set(encoderFlags, "rc", "vbr", 0);
      av_dict_set(encoderFlags, "spatial_aq", "1", 0);
      av_dict_set(encoderFlags, "aq-strength",  "15", 0);
      av_dict_set(encoderFlags, "delay", "0", 0);
      videoEncoder.initialize(encoderFlags);
      av_dict_free(encoderFlags);
      encoderFlags.close();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "WebcamUDPStreamingShutdown"));

      run();
   }

   private void run()
   {
      Mat frame = new Mat();
      while (!done && !shutdown)
      {
         // Read a frame
         videoCapture.read(frame);

         // Encode it and send it off
         videoEncoder.setNextFrame(frame);
         done = !videoEncoder.encodeAndWriteNextFrame();
      }

      readyForShutdown.set();
   }

   private void destroy()
   {
      shutdown = true;

      readyForShutdown.blockingPoll();

      videoEncoder.destroy();
      avio_closep(ioContext);
      ioContext.close();
      avformat_free_context(outputContext);
      outputContext.close();
      videoCapture.close();
   }

   public static void main(String[] args)
   {
      new FFMPEGWebcamUDPStreamingDemo();
   }
}
