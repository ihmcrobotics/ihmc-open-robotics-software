package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;

import static org.bytedeco.ffmpeg.global.avcodec.av_packet_rescale_ts;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

/**
 * FFMPEG demo which reads images from a webcam and streams an H.264 encoded video over HTTP.
 * To view the video, use the following command:
 * {@code ffplay http://127.0.0.1:8080 -fflags nobuffer}
 */
public class FFMPEGWebcamHTTPStreamingDemo
{
   private final VideoCapture videoCapture;
   private final Mat image = new Mat();
   private final TypedNotification<Mat> imageNotification = new TypedNotification<>();

   private FFMPEGWebcamHTTPStreamingDemo()
   {
      int error;

      // Open webcam
      videoCapture = new VideoCapture(-1);

      // Start webcam capture thread
      ThreadTools.startAThread(this::captureImage, "ImageCapture");

      // Get image info
      int imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
      double reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

      // Make server listen for clients
      AVDictionary serverOptions = new AVDictionary();
      error = av_dict_set(serverOptions, "listen", "2", 0);
      FFMPEGTools.checkNegativeError(error, "Setting server options to listen");

      // Open the server
      AVIOContext serverContext = new AVIOContext();
      error = avio_open2(serverContext, "http://127.0.0.1:8080", AVIO_FLAG_WRITE, null, serverOptions);
      FFMPEGTools.checkError(error, serverContext, "Opening server");

      for (int i = 0; true; i++)
      {
         // Listen for clients
         LogTools.info("Waiting for client...");
         AVIOContext clientContext = new AVIOContext();
         error = avio_accept(serverContext, clientContext); // this is blocking until we get a client\
         FFMPEGTools.checkError(error, clientContext, "Accepting client");

         // Start a client handler thread
         LogTools.info("Got client #{}", i);
         ThreadTools.startAThread(new ClientHandler(clientContext, imageWidth, imageHeight, reportedFPS), "ClientThread" + i);
      }
   }

   private void captureImage()
   {
      while (true)
      {
         videoCapture.read(image);
         imageNotification.set(image.clone());
      }
   }

   private class ClientHandler implements Runnable
   {
      private final AVFormatContext outputContext;
      private final AVStream outputStream;
      private final FFMPEGVideoEncoder videoEncoder;
      private int error;
      private boolean keepGoing = true;
      private boolean disconnected = false;

      private ClientHandler(AVIOContext clientContext, int imageWidth, int imageHeight, double outputFrameRate)
      {
         // Create an output context for the client
         outputContext = new AVFormatContext();
         error = avformat_alloc_output_context2(outputContext, null, "h264", null);
         FFMPEGTools.checkError(error, outputContext, "Allocating output context");
         outputContext.pb(clientContext);

         // Initialize the video encoder
         videoEncoder = new FFMPEGVideoEncoder(outputContext.oformat(),
                                               "h264_nvenc",
                                               400000,
                                               imageWidth,
                                               imageHeight,
                                               AV_PIX_FMT_YUV420P,
                                               outputFrameRate,
                                               10,
                                               2,
                                               AV_PIX_FMT_BGR24);

         // Set flags to make encoding low latency
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

         outputStream = videoEncoder.newStream(outputContext);
      }

      @Override
      public void run()
      {
         // Write the header
         LogTools.info("Sending header...");
         AVDictionary streamFlags = new AVDictionary();
         avutil.av_dict_set(streamFlags, "lossless", "0", 0);
         error = avformat_write_header(outputContext, streamFlags);
         LogTools.info("Sent header");

         while (keepGoing && !disconnected)
         {
            // Get most recent image
            Mat image = imageNotification.blockingPoll();
            if (image == null)
               continue;

            // Encode the image and send the packet to client
            videoEncoder.setNextFrame(image);
            keepGoing = videoEncoder.encodeNextFrame(packet ->
            {
               av_packet_rescale_ts(packet, packet.time_base(), outputStream.time_base());
               packet.stream_index(outputStream.index());

               error = av_interleaved_write_frame(outputContext, packet);
               if (error < 0)
                  disconnected = true;
            });
         }

         LogTools.info("Closing connection with client");
         av_write_trailer(outputContext);

         videoEncoder.destroy();
         avio_closep(outputContext.pb());
         outputContext.pb().close();

         avformat_free_context(outputContext);
         outputContext.close();

         av_dict_free(streamFlags);
         streamFlags.close();
      }
   }

   public static void main(String[] args)
   {
      new FFMPEGWebcamHTTPStreamingDemo();
   }
}
