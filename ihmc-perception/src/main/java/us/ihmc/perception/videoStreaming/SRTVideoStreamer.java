package us.ihmc.perception.videoStreaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.ffmpeg.FFMPEGTools;
import us.ihmc.perception.ffmpeg.FFMPEGVideoEncoder;

import java.net.InetSocketAddress;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import static java.util.Map.entry;
import static org.bytedeco.ffmpeg.global.avformat.av_guess_format;
import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_YUV420P;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class SRTVideoStreamer
{
   private static final String OUTPUT_FORMAT_NAME = "mpegts";
   private static final String PREFERRED_CODEC = "hevc_nvenc";
   private static final int OUTPUT_PIXEL_FORMAT = AV_PIX_FMT_YUV420P;

   /**
    * For available options, see <a href="https://www.ffmpeg.org/ffmpeg-protocols.html#srt">FFMPEG srt documentation.</a>
    * To get a decent SRT configuration, see <a href="https://srtlab.github.io/srt-cookbook/protocol/configuration.html">SRT Configuration Calculator</a>
    */
   private static final Map<String, String> SRT_IO_OPTIONS
         = Map.ofEntries(entry("mode", "listener"),
                         entry("transtype", "live"),
                         entry("rcvlatency", "0"),
                         entry("peerlatency", "0"),
                         entry("mss", "1360"),
                         entry("payload_size", "1316"),
                         entry("fc", "42"),
                         entry("rcvbuf", "55944"));

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

   private final AVDictionary ioOptions;
   private final AVOutputFormat outputFormat;

   private final Queue<InetSocketAddress> connectionQueue = new LinkedBlockingQueue<>();
   private final Map<InetSocketAddress, SRTStreamWriter> callers;

   private final AVDictionary encoderOptions;
   private FFMPEGVideoEncoder encoder;

   /**
    * (1) Construct the streamer
    */
   public SRTVideoStreamer()
   {
      ioOptions = new AVDictionary();
      FFMPEGTools.setAVDictionary(ioOptions, SRT_IO_OPTIONS);

      encoderOptions = new AVDictionary();
      FFMPEGTools.setAVDictionary(encoderOptions, HEVC_NVENC_OPTIONS);

      outputFormat = av_guess_format(OUTPUT_FORMAT_NAME, null, null);
      callers = new HashMap<>();
   }

   /**
    * (2) Initialize the streamer
    * @param imageWidth Witdh of images being streamed
    * @param imageHeight Height of images being stream
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
   }

   public void setNextFrame(RawImage image)
   {
      if (image.get() == null)
         return;

      setNextFrame(image.getCpuImageMat());

      image.release();
   }

   public void setNextFrame(Mat image)
   {
      encoder.setNextFrame(image); // TODO: Use GpuMat instead
      encoder.encodeNextFrame(this::writeToCallers);
   }

   private void writeToCallers(AVPacket packetToWrite)
   {
      boolean writeSucceeded;

      Iterator<SRTStreamWriter> callerIterator = callers.values().iterator();
      while (callerIterator.hasNext())
      {
         SRTStreamWriter callerWriter = callerIterator.next();
         if (!callerWriter.isConnected())
            continue;

         writeSucceeded = callerWriter.write(packetToWrite);
         if (!writeSucceeded)
         {
            callerWriter.destroy();
            callerIterator.remove();
         }
      }
   }

   /**
    * (3*) Add callers.
    * This can happen any time.
    * @param callerAddress Address of caller to add
    */
   public void queueCallerToConnect(InetSocketAddress callerAddress)
   {
      connectionQueue.add(callerAddress);
   }

   /**
    * (4*) Connect to newly added callers.
    * Since establishing the connection with a caller can take some time,
    * it is best to call this in a separate thread.
    * This can happen any time.
    */
   public void connectToNewCallers()
   {
      while (!connectionQueue.isEmpty())
      {
         InetSocketAddress callerAddress = connectionQueue.poll();
         SRTStreamWriter callerOutput = new SRTStreamWriter(encoder, callerAddress, outputFormat, ioOptions, null);
         if (callerOutput.connect()) // This call can block for a while
            callers.putIfAbsent(callerAddress, callerOutput);
      }
   }

   public void removeCaller(InetSocketAddress callerAddress)
   {
      if (callers.containsKey(callerAddress))
         callers.remove(callerAddress).destroy();

      connectionQueue.remove(callerAddress);
   }

   public int connectedCallerCount()
   {
      return callers.size();
   }

   public int queuedCallerCount()
   {
      return connectionQueue.size();
   }

   public int totalCallerCount()
   {
      return connectedCallerCount() + queuedCallerCount();
   }

   public void destroy()
   {
      av_dict_free(ioOptions);
      ioOptions.close();

      av_dict_free(encoderOptions);
      encoderOptions.close();

      if (encoder != null)
         encoder.destroy();

      for (SRTStreamWriter callerOutput : callers.values())
         callerOutput.destroy();
   }
}
