package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ffmpeg.FFMPEGInterruptCallback;
import us.ihmc.perception.ffmpeg.FFMPEGTools;
import us.ihmc.perception.ffmpeg.FFMPEGVideoEncoder;

import java.net.InetSocketAddress;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import static java.util.Map.entry;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_YUV420P;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

// TODO: Make abstract SRTStreamer class and extend to video and audio
public class SRTVideoStreamer
{
   private static final String OUTPUT_FORMAT_NAME = "mpegts";
   private static final String PREFERRED_CODEC = "hevc_nvenc"; // TODO: use other codec if CUDA not available
   private static final int OUTPUT_PIXEL_FORMAT = AV_PIX_FMT_YUV420P;

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

   private final AVDictionary encoderOptions;
   private FFMPEGVideoEncoder encoder;

   private final AVOutputFormat outputFormat;
   private final FFMPEGInterruptCallback interruptCallback;

   private final Thread callerConnector;
   private final Map<String, String> liveSRTOptions;
   private final String srtAddress;
   private final Set<SRTStreamWriter> callers = ConcurrentHashMap.newKeySet();

   private volatile boolean running = true;
   private boolean initialized = false;

   /**
    * Constructs a streamer, automatically detecting an address and available port
    */
   public SRTVideoStreamer()
   {
      this(StreamingTools.getMyAddress());
   }

   /**
    * Constructs a streamer, specifying the IP address and socket to output the stream on.
    * @param outputAddress IPv4 address and socket to output the stream on.
    */
   public SRTVideoStreamer(InetSocketAddress outputAddress)
   {
      srtAddress = StreamingTools.toSRTAddress(outputAddress);

      interruptCallback = new FFMPEGInterruptCallback();

      liveSRTOptions = StreamingTools.getLiveSRTOptions();
      liveSRTOptions.put("mode", "listener");

      encoderOptions = new AVDictionary();
      FFMPEGTools.setAVDictionary(encoderOptions, HEVC_NVENC_OPTIONS);

      outputFormat = av_guess_format(OUTPUT_FORMAT_NAME, null, null);

      callerConnector = ThreadTools.startAsDaemon(this::connectToCallers, getClass().getSimpleName() + "CallerConnector");
   }

   /**
    * Initialize the streamer to encode images
    * @param imageWidth Width of images being streamed
    * @param imageHeight Height of images being streamed
    * @param inputFPS FPS at which images will be provided
    * @param inputAVPixelFormat Pixel format of images being provided (Must be one of AV_PIX_FMT_*)
    */
   public void initialize(int imageWidth, int imageHeight, double inputFPS, int inputAVPixelFormat)
   {
      int bitRate = imageWidth * imageHeight;
      int gopSize = (int) inputFPS / 10; // Send key frames every 0.1 seconds

      encoder = new FFMPEGVideoEncoder(outputFormat,
                                       PREFERRED_CODEC,
                                       bitRate,
                                       imageWidth,
                                       imageHeight,
                                       OUTPUT_PIXEL_FORMAT,
                                       inputFPS,
                                       gopSize,
                                       0,
                                       inputAVPixelFormat);
      encoder.initialize(encoderOptions);
      initialized = true;
   }

   public void sendFrame(Mat frame)
   {
      encoder.setNextFrame(frame); // TODO: Use GpuMat instead
      encoder.encodeNextFrame(this::writeToCallers);
   }

   private void writeToCallers(AVPacket packetToWrite)
   {
      boolean writeSucceeded;

      Iterator<SRTStreamWriter> callerIterator = callers.iterator();
      while (callerIterator.hasNext())
      {
         SRTStreamWriter callerWriter = callerIterator.next();

         if (!callerWriter.isConnected())
         {
            callerWriter.destroy();
            callerIterator.remove();
            continue;
         }

         writeSucceeded = callerWriter.write(packetToWrite);
         if (!writeSucceeded)
         {
            callerWriter.destroy();
            callerIterator.remove();
         }
      }
   }

   public int connectedCallerCount()
   {
      return callers.size();
   }

   public boolean isInitialized()
   {
      return initialized;
   }

   public void destroy()
   {
      LogTools.debug("Shutting down {}", getClass().getSimpleName());
      running = false;

      interruptCallback.interrupt();
      try
      {
         callerConnector.join();
      }
      catch (InterruptedException e)
      {
         interruptCallback.interrupt();
      }

      Iterator<SRTStreamWriter> callerIterator = callers.iterator();
      while (callerIterator.hasNext())
      {
         callerIterator.next().destroy();
         callerIterator.remove();
      }

      av_dict_free(encoderOptions);
      encoderOptions.close();

      if (encoder != null)
         encoder.destroy();

      interruptCallback.close();
   }

   private void connectToCallers()
   {
      LogTools.debug("Starting {} thread on {}", callerConnector.getName(), srtAddress);

      while (running)
      {
         // Set the SRT options
         AVDictionary srtOptions = new AVDictionary();
         FFMPEGTools.setAVDictionary(srtOptions, liveSRTOptions);

         // Wait for caller connection
         LogTools.debug("Waiting for connection on {}", srtAddress);
         AVIOContext callerSRTContext = new AVIOContext();
         int error = avio_open2(callerSRTContext, srtAddress, AVIO_FLAG_WRITE, interruptCallback, srtOptions);

         // Check whether connection succeeded
         if (error >= 0)
         {
            // Ensure options are set correctly
            FFMPEGTools.checkDictionaryAfterUse(srtOptions);

            LogTools.debug("Got a connection on {}", srtAddress);
            SRTStreamWriter callerWriter = new SRTStreamWriter(encoder, callerSRTContext, outputFormat, null);
            if (callerWriter.startOutput())
               callers.add(callerWriter);
            else
               callerWriter.destroy();
         }

         av_dict_free(srtOptions);
         srtOptions.close();
      }

      LogTools.debug("{} thread listening on {} ended", callerConnector.getName(), srtAddress);
   }
}
