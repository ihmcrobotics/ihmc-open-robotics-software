package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFMPEGWebcamStreamingDemo
{
   private final VideoCapture videoCapture;
   private final Mat image = new Mat();
   private final TypedNotification<Mat> imageNotification = new TypedNotification<>();
   private long sequenceNumber = 0L;

   public FFMPEGWebcamStreamingDemo()
   {
      int error;

      videoCapture = new VideoCapture(-1);

      ThreadTools.startAThread(this::captureImage, "ImageCapture");

      int imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
      double reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

      AVDictionary ioOptions = new AVDictionary();
      error = av_dict_set(ioOptions, "listen", "2", 0);
      FFMPEGTools.checkNegativeError(error, "Setting ioOption listen");

      AVIOContext serverContext = new AVIOContext();
      error = avio_open2(serverContext, "http://127.0.0.1:8080", AVIO_FLAG_WRITE, null, ioOptions);
      FFMPEGTools.checkNegativeError(error, "Opening IO");
      FFMPEGTools.checkPointer(serverContext, "Opening IO");

      for (int i = 0; true; i++)
      {
         LogTools.info("Waiting for client...");
         AVIOContext clientContext = new AVIOContext();
         error = avio_accept(serverContext, clientContext);
         FFMPEGTools.checkNegativeError(error, "Accepting client");
         FFMPEGTools.checkPointer(clientContext, "Accepting client");

         LogTools.info("Got a client!");
         ThreadTools.startAThread(new ClientHandler(clientContext, imageWidth, imageHeight, reportedFPS), "ClientThread" + i);
      }
   }

   public void captureImage()
   {
      while (true)
      {
         videoCapture.read(image);
         sequenceNumber++;
         imageNotification.set(image.clone());
      }
   }

   private class ClientHandler implements Runnable
   {
      private final AVFormatContext outputContext;
      private final FFMPEGVideoEncoder videoEncoder;
      private int error;
      private boolean keepGoing = true;
      private boolean disconnected = false;

      private ClientHandler(AVIOContext clientContext, int imageWidth, int imageHeight, double outputFrameRate)
      {
         outputContext = new AVFormatContext();
         error = avformat_alloc_output_context2(outputContext, null, "h264", null);
         FFMPEGTools.checkError(error, outputContext, "Allocating output context");
         outputContext.pb(clientContext);

         videoEncoder = new FFMPEGVideoEncoder(outputContext,
                                               "h264_nvenc",
                                               400000,
                                               imageWidth,
                                               imageHeight,
                                               AV_PIX_FMT_YUV420P,
                                               outputFrameRate,
                                               10,
                                               2,
                                               AV_PIX_FMT_BGR24);
         videoEncoder.initialize();
      }

      @Override
      public void run()
      {
         LogTools.info("Sending header...");
         AVDictionary streamFlags = new AVDictionary();
         avutil.av_dict_set(streamFlags, "lossless", "0", 0);
         error = avformat_write_header(outputContext, streamFlags);
         LogTools.info("Sent header");

         while (keepGoing && !disconnected)
         {
            Mat image = imageNotification.blockingPoll();
            LogTools.info("Sending frame {}", sequenceNumber);

            videoEncoder.setNextFrame(image);
            keepGoing = videoEncoder.encodeNextFrame(packet ->
            {
               error = av_interleaved_write_frame(outputContext, packet);
               if (error < 0)
                  disconnected = true;
            });
         }

         LogTools.info("Closing connection with client");
         av_write_trailer(outputContext);

         videoEncoder.destroy();
         avio_closep(outputContext.pb());

         avformat_free_context(outputContext);

         av_dict_free(streamFlags);
         streamFlags.close();
      }
   }

   public static void main(String[] args)
   {
      new FFMPEGWebcamStreamingDemo();
   }
}
