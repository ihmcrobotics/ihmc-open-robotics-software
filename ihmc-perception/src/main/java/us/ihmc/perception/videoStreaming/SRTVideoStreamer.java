package us.ihmc.perception.videoStreaming;

import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.ffmpeg.FFMPEGTools;
import us.ihmc.perception.ffmpeg.FFMPEGVideoEncoder;
import us.ihmc.perception.opencv.OpenCVTools;

import java.util.Map;

import static java.util.Map.entry;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class SRTVideoStreamer
{
   private static final String OUTPUT_FORMAT = "mpegts";
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
                         entry("gpu", "-1"),          // Use any GPU
                         entry("delay", "0"),         // No delay to frame output
                         entry("spatial-aq", "1"),    // Enable spatial AQ
                         entry("zerolatency", "1"),   // Don't introduce reordering delay
                         entry("aq-strength", "15"),  // Most aggressive AQ
                         entry("cq", "0"),            // Quality level (0 = auto, 1 = nearly lossless, 51 = low quality)
                         entry("multipass", "0"));    // Disable multipass

   private final AVDictionary ioOptions;
   private final SRTStreamOutputList outputList;

   private final AVDictionary encoderOptions;
   private FFMPEGVideoEncoder encoder;

   private final double inputFPS;
   private final int inputPixelFormat;

   public SRTVideoStreamer(double inputFPS, int inputAVPixelFormat)
   {
      this.inputFPS = inputFPS;
      this.inputPixelFormat = inputAVPixelFormat;

      ioOptions = new AVDictionary();
      FFMPEGTools.setAVDictionary(ioOptions, SRT_IO_OPTIONS);

      encoderOptions = new AVDictionary();
      FFMPEGTools.setAVDictionary(encoderOptions, HEVC_NVENC_OPTIONS);

      outputList = new SRTStreamOutputList(OUTPUT_FORMAT, ioOptions, null);
   }

   public void setNextFrame(RawImage image)
   {
      if (image.get() == null)
         return;

      if (encoder == null)
         initialize(image);

      encoder.setNextFrame(image.getCpuImageMat()); // TODO: Use GpuMat instead
      encoder.encodeNextFrame(outputList::writeToAll);
   }

   private void initialize(RawImage firstImage)
   {
      Mat imageMat = firstImage.getCpuImageMat();
      int bitRate = (int) OpenCVTools.dataSize(imageMat) / 10;
      int gopSize = (int) inputFPS / 10;

      encoder = new FFMPEGVideoEncoder(outputList.getExampleFormatContext(),
                                       PREFERRED_CODEC,
                                       bitRate,
                                       firstImage.getImageWidth(),
                                       firstImage.getImageHeight(),
                                       OUTPUT_PIXEL_FORMAT,
                                       inputFPS,
                                       gopSize,
                                       0,
                                       inputPixelFormat);
      encoder.initialize(encoderOptions);
   }

   public SRTStreamOutputList getStreamOutputList()
   {
      return outputList;
   }

   public void close()
   {
      av_dict_free(ioOptions);
      ioOptions.close();

      av_dict_free(encoderOptions);
      encoderOptions.close();

      if (encoder != null)
         encoder.destroy();

      outputList.close();
   }
}
