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
import us.ihmc.log.LogTools;
import us.ihmc.perception.streaming.StreamingTools;

import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

/**
 * <p>
 * Demo for streaming a webcam video over SRT using LibAV (backend library of FFmpeg).
 * </p>
 * <p>
 * To view the stream, use the following command: {@code ffplay srt://127.0.0.1:60001 -fflags nobuffer}.
 * It is also possible to view the stream using VLC media player by opening a Network Stream (Ctrl + N),
 * and entering "srt://127.0.0.1:60001" as the url.
 * </p>
 */
public class FFmpegWebcamSRTStreamingDemo
{
   private static final String SERVER_ADDRESS = "srt://127.0.0.1:60001";

   private final VideoCapture webcam;
   private final Mat image = new Mat();
   private final Thread imageCaptureThread;

   private final FFmpegVideoEncoder videoEncoder;
   private final FFmpegInterruptCallback interruptCallback = new FFmpegInterruptCallback();

   private final Set<CallerHandler> callers = ConcurrentHashMap.newKeySet();

   private final AtomicBoolean running = new AtomicBoolean(true);

   private FFmpegWebcamSRTStreamingDemo()
   {
      int error;
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Destruction"));

      // Open webcam
      webcam = new VideoCapture(-1);

      // Get image info
      int imageWidth = (int) webcam.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) webcam.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);

      // Create an encoder
      AVOutputFormat outputFormat = av_guess_format("mpegts", null, null);
      FFmpegTools.checkPointer(outputFormat, "Guessing output format");

      videoEncoder = new FFmpegSoftwareVideoEncoder(outputFormat,
                                                    "hevc_nvenc",
                                                    10 * imageWidth * imageHeight,
                                                    imageWidth,
                                                    imageHeight,
                                                    AV_PIX_FMT_YUV420P,
                                                    10,
                                                    0,
                                                    AV_PIX_FMT_BGR24);
      videoEncoder.initialize();

      // Start webcam capture thread
      imageCaptureThread = ThreadTools.startAsDaemon(this::captureImage, "ImageCapture");

      for (int i = 0; ; i++)
      {
         // Make server listen for callers
         AVDictionary serverOptions = new AVDictionary();
         Map<String, String> srtOptions = StreamingTools.getLiveSRTOptions();
         srtOptions.put("mode", "listener");
         FFmpegTools.setAVDictionary(serverOptions, srtOptions);


         // Listen for caller
         LogTools.info("Listening for callers on {}", SERVER_ADDRESS);
         AVIOContext srtContext = new AVIOContext();
         error = avio_open2(srtContext, SERVER_ADDRESS, AVIO_FLAG_WRITE, interruptCallback, serverOptions);
         if (error >= 0)
         {  // Add caller to list of callers
            LogTools.info("Got caller #{}", i);
            callers.add(new CallerHandler(videoEncoder, srtContext, outputFormat));
         }

         av_dict_free(serverOptions);
         serverOptions.close();
      }
   }

   private void captureImage()
   {
      while (running.get())
      {
         webcam.read(image);
         videoEncoder.setNextFrame(image);
         videoEncoder.encodeNextFrame(encodedPacket ->
         {
            Iterator<CallerHandler> callerIterator = callers.iterator();
            while (callerIterator.hasNext())
            {
               CallerHandler callerHandler = callerIterator.next();

               if (!callerHandler.write(encodedPacket))
               {
                  callerHandler.destroy();
                  callerIterator.remove();
                  LogTools.info("Disconnected from a caller");
               }
            }
         });
      }
   }

   private void destroy()
   {
      if (!running.compareAndSet(true, false))
         return;

      interruptCallback.interrupt();
      try
      {
         imageCaptureThread.join();
      }
      catch (InterruptedException ignored) {}

      Iterator<CallerHandler> callerIterator = callers.iterator();
      while (callerIterator.hasNext())
      {
         callerIterator.next().destroy();
         callerIterator.remove();
      }

      videoEncoder.destroy();
      interruptCallback.close();
   }

   private class CallerHandler
   {
      private final FFmpegVideoEncoder encoder;
      private final AVIOContext srtContext;

      private final AVFormatContext outputFormatContext;
      private final AVPacket packetCopy;
      private final AVStream outputStream;

      private boolean connected;
      private int error;

      private CallerHandler(FFmpegVideoEncoder encoder, AVIOContext srtContext, AVOutputFormat outputFormat)
      {
         this.encoder = encoder;
         this.srtContext = srtContext;

         outputFormatContext = new AVFormatContext();

         packetCopy = av_packet_alloc();
         FFmpegTools.checkPointer(packetCopy, "Allocating a packet");

         // Start the output
         error = avformat_alloc_output_context2(outputFormatContext, outputFormat, (String) null, null);
         outputFormatContext.pb(srtContext);
         outputFormatContext.interrupt_callback(interruptCallback);

         outputStream = encoder.newStream(outputFormatContext);

         error = avformat_write_header(outputFormatContext, (AVDictionary) null);

         connected = true;
      }

      private boolean write(AVPacket packetToWrite)
      {
         if (connected)
         {
            av_packet_ref(packetCopy, packetToWrite);

            av_packet_rescale_ts(packetCopy, encoder.getTimeBase(), outputStream.time_base());
            packetCopy.stream_index(outputStream.index());
            error = av_interleaved_write_frame(outputFormatContext, packetCopy);
            if (error < 0)
               connected = false;

            av_packet_unref(packetCopy);
         }

         return connected;
      }

      private void destroy()
      {
         if (connected)
         {
            av_write_trailer(outputFormatContext);
            connected = false;
         }

         avio_closep(srtContext);
         srtContext.close();

         av_packet_free(packetCopy);
         packetCopy.close();

         avformat_free_context(outputFormatContext);
         outputFormatContext.close();

         outputStream.close();
      }
   }

   public static void main(String[] args)
   {
      FFmpegWebcamSRTStreamingDemo demo = new FFmpegWebcamSRTStreamingDemo();
      demo.destroy();
   }
}
