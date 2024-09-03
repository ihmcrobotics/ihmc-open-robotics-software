package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;

import java.net.URI;
import java.net.URISyntaxException;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR24;

public class FFMPEGVideoDecodingDemo
{
   private static final String VIDEO_FILE_NAME = "example.webm";

   private final AVFormatContext inputContext;
   private final FFMPEGVideoDecoder videoDecoder;

   private boolean shutdown = false;
   private final Notification readyForShutdown = new Notification();

   private FFMPEGVideoDecodingDemo()
   {
      URI videoPath;
      try
      {
         videoPath = FFMPEGVideoDecodingDemo.class.getResource(VIDEO_FILE_NAME).toURI();
      }
      catch (URISyntaxException exception)
      {
         throw new RuntimeException("Failed to find video file", exception);
      }

      inputContext = avformat_alloc_context();
      avformat_open_input(inputContext, videoPath.getPath(), null, null);
      avformat_find_stream_info(inputContext, (AVDictionary) null);

      videoDecoder = new FFMPEGVideoDecoder(inputContext, AV_PIX_FMT_BGR24);
      videoDecoder.initialize(null, this::getNextPacket);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "WebcamUDPStreamingShutdown"));

      run();
   }

   private int getNextPacket(AVPacket packetToPack)
   {
      return av_read_frame(inputContext, packetToPack);
   }

   private void run()
   {
      while (!shutdown)
      {
         Mat frame = videoDecoder.getNextFrame();
         if (frame == null)
            break;

         opencv_highgui.imshow("Decoded Video", frame);
         opencv_highgui.waitKey(1);
         frame.close();
      }

      readyForShutdown.set();
   }

   private void destroy()
   {
      shutdown = true;

      readyForShutdown.blockingPoll();

      videoDecoder.destroy();
      avformat_free_context(inputContext);
      inputContext.close();
   }

   public static void main(String[] args)
   {
      new FFMPEGVideoDecodingDemo();
   }
}
