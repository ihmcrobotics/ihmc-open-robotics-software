package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.perception.streaming.StreamingTools;

import java.util.Map;

import static java.util.Map.entry;
import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFmpegWebcamSRTStreamingDemo
{
   /** hevc_nvenc options can be found using {@code ffmpeg -hide_banner -h encoder=hevc_nvenc}. */
   private static final Map<String, String> HEVC_NVENC_OPTIONS
         = Map.ofEntries(entry("preset", "p1"),       // p1 = fastest, p2 = fast, p3 = medium ... p7 = slowest
                         entry("tune", "ull"),        // "Ultra low latency"
                         entry("level", "auto"),      // Encoding level restriction
                         entry("rc", "vbr"),          // Rate control: variable bitrate mode
                         entry("gpu", "any"),         // Use any GPU
                         entry("delay", "0"),         // No delay to frame output
                         entry("zerolatency", "1"),   // Don't introduce reordering delay
                         entry("cq", "0"),            // Quality level (0 = auto, 1 = nearly lossless, 51 = low quality)
                         entry("multipass", "0"));    // Disable multipass

   private final VideoCapture webcam;
   private final Mat image = new Mat();
   private final TypedNotification<Mat> imageNotification = new TypedNotification<>();

   private FFmpegWebcamSRTStreamingDemo()
   {
      int error;

      // Open webcam
      webcam = new VideoCapture(-1);

      // Start webcam capture thread
      ThreadTools.startAsDaemon(this::captureImage, "ImageCapture");

      // Get image info
      int imageWidth = (int) webcam.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) webcam.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);

      for (int i = 0; ; i++)
      {
         // Make server listen for callers
         AVDictionary serverOptions = new AVDictionary();
         Map<String, String> srtOptions = StreamingTools.getLiveSRTOptions();
         srtOptions.put("mode", "listener");
         FFmpegTools.setAVDictionary(serverOptions, srtOptions);

         // Listen for caller
         AVIOContext serverContext = new AVIOContext();

         error = avio_open2(serverContext, "srt://127.0.0.1:60001", AVIO_FLAG_WRITE, null, serverOptions);
         FFmpegTools.checkNegativeError(error, "Opening Connection");

         ThreadTools.startAThread(new CallerHandler(serverContext, imageWidth, imageHeight), "CallerThread" + i);

         av_dict_free(serverOptions);
         serverOptions.close();
      }
   }

   private void captureImage()
   {
      while (true)
      {
         webcam.read(image);
         imageNotification.set(image.clone());
      }
   }

   private class CallerHandler implements Runnable
   {
      private final AVFormatContext outputContext;
      private final AVStream outputStream;
      private final AVPacket packetCopy;
      private final FFmpegSoftwareVideoEncoder videoEncoder;
      private int error;
      private boolean keepGoing = true;
      private boolean disconnected = false;

      private CallerHandler(AVIOContext serverContext, int imageWidth, int imageHeight)
      {
         AVOutputFormat outputFormat = av_guess_format("h264", null, null);

         outputContext = new AVFormatContext();
         error = avformat_alloc_output_context2(outputContext, outputFormat, (String) null, null);
         FFmpegTools.checkError(error, outputContext, "Allocating output context");
         outputContext.pb(serverContext);

         packetCopy = av_packet_alloc();
         FFmpegTools.checkPointer(packetCopy, "Allocating a packet");

         videoEncoder = new FFmpegSoftwareVideoEncoder(outputFormat,
                                                       "hevc_nvenc",
                                               imageWidth * imageHeight,
                                                       imageWidth,
                                                       imageHeight,
                                                       AV_PIX_FMT_YUV420P,
                                                       10,
                                                       2,
                                                       AV_PIX_FMT_BGR24);

         AVDictionary encoderOptions = new AVDictionary();
         FFmpegTools.setAVDictionary(encoderOptions, HEVC_NVENC_OPTIONS);

         videoEncoder.initialize(encoderOptions);

         av_dict_free(encoderOptions);
         encoderOptions.close();

         outputStream = videoEncoder.newStream(outputContext);
      }

      @Override
      public void run()
      {
         // Write the header
         LogTools.info("Sending header...");
         error = avformat_write_header(outputContext, (AVDictionary) null);
         LogTools.info("Sent header");

         while (keepGoing && !disconnected)
         {
            // Get most recent image
            Mat image = imageNotification.blockingPoll();

            if (image == null || image.isNull())
               continue;

            // Encode the image and send the packet to client
            videoEncoder.setNextFrame(image);
            keepGoing = videoEncoder.encodeNextFrame(packet ->
            {
               av_packet_ref(packetCopy, packet);

               av_packet_rescale_ts(packetCopy, videoEncoder.getTimeBase(), outputStream.time_base());
               packetCopy.stream_index(outputStream.index());
               error = av_interleaved_write_frame(outputContext, packetCopy);
               if (error < 0)
                  disconnected = true;

               av_packet_unref(packetCopy);
            });
         }

         LogTools.info("Closing connection with client");
         av_write_trailer(outputContext);

         videoEncoder.destroy();
         avio_closep(outputContext.pb());
         outputContext.pb().close();

         avformat_free_context(outputContext);
         outputContext.close();
      }
   }

   public static void main(String[] args)
   {
      new FFmpegWebcamSRTStreamingDemo();
   }
}
