package us.ihmc.perception.demo;

import org.apache.commons.lang3.mutable.MutableLong;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.ffmpeg.FFmpegHardwareVideoEncoder;
import us.ihmc.perception.ffmpeg.FFmpegSoftwareVideoEncoder;
import us.ihmc.perception.ffmpeg.FFmpegTools;
import us.ihmc.perception.ffmpeg.FFmpegVideoEncoder;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.streaming.StreamingTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO.RecordMode;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Map;

import static org.bytedeco.ffmpeg.global.avformat.av_guess_format;
import static org.bytedeco.ffmpeg.global.avutil.*;

/**
 * Runs various compression algorithms on images grabbed from an SVO file.
 * Calculates the average compressed size, compression ratio, amd compression duration.
 * Outputs results to a .csv file in the Documents folder.
 */
public class SVOImageCompressionAlgorithmComparison
{
   private static final int DATA_POINTS = 500;
   private static final String SVO_FILE_NAME = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                        .toAbsolutePath()
                                                                                        .toString();
   private final ROS2Node ros2Node = ROS2Tools.createLoopbackROS2Node(PubSubImplementation.FAST_RTPS, "compression_algorithm_comparison");
   private final ZEDColorDepthImageRetrieverSVO zedDataRetriever;

   private final ImageCompressionAlgorithmComparison algorithmComparison = new ImageCompressionAlgorithmComparison();

   private final CUDAJPEGProcessor cudaJpegProcessor = new CUDAJPEGProcessor();
   private final CUDACompressionTools compressionTools = new CUDACompressionTools();
   private FFmpegVideoEncoder colorEncoder;
   private FFmpegVideoEncoder depthEncoder;

   private SVOImageCompressionAlgorithmComparison()
   {
      algorithmComparison.addColorCompressionAlgorithm("NVJPEG", this::nvjpegColorCompression);
      algorithmComparison.addColorCompressionAlgorithm("NVCOMP", this::nvcompColorCompression);
      algorithmComparison.addDepthCompressionAlgorithm("NVCOMP", this::nvcompHybridDepthCompression);
      algorithmComparison.addColorCompressionAlgorithm("FFmpeg", this::ffmpegColorEncoding);
      algorithmComparison.addDepthCompressionAlgorithm("FFmpeg", this::ffmpegDepthEncoding);
      algorithmComparison.addColorCompressionAlgorithm("OpenCVTools", this::openCVColorCompression);
      algorithmComparison.addDepthCompressionAlgorithm("OpenCVTools", this::openCVDepthCompression);

      zedDataRetriever = new ZEDColorDepthImageRetrieverSVO(0, () -> true, () -> true, new ROS2Helper(ros2Node), RecordMode.PLAYBACK, SVO_FILE_NAME);
      zedDataRetriever.grabOneFrame(); // initializes retriever
   }

   private void initialize(RawImage colorImage, RawImage depthImage)
   {
      LogTools.info("Initializing...");

      int bitRate = 10 * colorImage.getWidth() * colorImage.getHeight();
      AVOutputFormat colorOutputFormat = av_guess_format("mpegts", null, null);
      AVOutputFormat depthOutputFormat = av_guess_format("matroska", null, null);

      // Initialize color encoder
      colorEncoder = new FFmpegHardwareVideoEncoder(colorOutputFormat,
                                                    "hevc_nvenc",
                                                    bitRate,
                                                    colorImage.getWidth(),
                                                    colorImage.getHeight(),
                                                    10,
                                                    0,
                                                    AV_PIX_FMT_BGR0);
      AVDictionary hevcOptions = new AVDictionary();
      FFmpegTools.setAVDictionary(hevcOptions, StreamingTools.getHEVCNVENCStreamingOptions());
      colorEncoder.initialize(hevcOptions);
      colorEncoder.setIntermediateColorConversion(opencv_imgproc.COLOR_BGR2BGRA);
      av_dict_free(hevcOptions);
      hevcOptions.close();

      // Initialize depth encoder
      depthEncoder = new FFmpegSoftwareVideoEncoder(depthOutputFormat,
                                                    "ffv1",
                                                    bitRate,
                                                    depthImage.getWidth(),
                                                    depthImage.getHeight(),
                                                    AV_PIX_FMT_GRAY16,
                                                    10,
                                                    0,
                                                    AV_PIX_FMT_GRAY16);
      AVDictionary ffv1Options = new AVDictionary();
      Map<String, String> options = StreamingTools.getFFV1StreamingOptions();
      FFmpegTools.setAVDictionary(ffv1Options, options);
      depthEncoder.initialize(ffv1Options);
      av_dict_free(ffv1Options);
      ffv1Options.close();
   }

   private void run() throws IOException
   {
      for (int i = 0; i < DATA_POINTS; ++i)
      {
         if (i % 10 == 0)
            LogTools.info("Grabbing frame {}", i);

         // Grab a frame and get the images
         zedDataRetriever.grabOneFrame();
         RawImage colorImage = zedDataRetriever.getLatestRawColorImage(RobotSide.LEFT);
         RawImage depthImage = zedDataRetriever.getLatestRawDepthImage();

         // Ensure all images are present both in host and device memory
         colorImage.getCpuImageMat();
         colorImage.getGpuImageMat();
         depthImage.getCpuImageMat();
         depthImage.getGpuImageMat();

         if (i == 0) // initialize on first run
            initialize(colorImage, depthImage);

         // Test the algorithms with color and depth
         algorithmComparison.test(colorImage);
         algorithmComparison.test(depthImage);

         colorImage.release();
         depthImage.release();
      }

      LogTools.info("Writing results to file...");
      String resultFileName = "CompressionAlgorithmComparison_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()) + ".csv";
      algorithmComparison.saveResultToFile(resultFileName);
      LogTools.info("Results save to ~/Documents/{}", resultFileName);
   }

   private void destroy()
   {
      LogTools.info("Finalizing...");
      cudaJpegProcessor.destroy();
      compressionTools.destroy();
      colorEncoder.destroy();
      depthEncoder.destroy();
      zedDataRetriever.destroy();
      ros2Node.destroy();
      LogTools.info("Finished!");
   }

   private long nvjpegColorCompression(Pointer cpuImage, Pointer gpuImage)
   {
      GpuMat image = new GpuMat(gpuImage);
      try (BytePointer encodedData = new BytePointer(OpenCVTools.dataSize(image)))
      {
         cudaJpegProcessor.encodeBGR(image, encodedData);
         return encodedData.limit();
      }
   }

   private long nvcompColorCompression(Pointer cpuImage, Pointer gpuImage)
   {
      try (BytePointer compressedData = compressionTools.compress(new GpuMat(gpuImage)))
      {
         return compressedData.limit();
      }
   }

   private long nvcompHybridDepthCompression(Pointer cpuImage, Pointer gpuImage)
   {
      try (BytePointer compressedData = compressionTools.compressDepth(new GpuMat(gpuImage)))
      {
         return compressedData.limit();
      }
   }

   private long ffmpegColorEncoding(Pointer cpuImage, Pointer gpuImage)
   {
      MutableLong totalSize = new MutableLong(0);
      colorEncoder.setNextFrame(gpuImage);
      colorEncoder.encodeNextFrame(packet -> totalSize.add(packet.size()));
      return totalSize.longValue();
   }

   private long ffmpegDepthEncoding(Pointer cpuImage, Pointer gpuImage)
   {
      MutableLong totalSize = new MutableLong(0);
      depthEncoder.setNextFrame(cpuImage);
      depthEncoder.encodeNextFrame(packet -> totalSize.add(packet.size()));
      return totalSize.longValue();
   }

   private long openCVColorCompression(Pointer cpuImage, Pointer gpuImage)
   {
      try (Mat yuvImage = new Mat();
           BytePointer compressedImage = new BytePointer())
      {
         OpenCVTools.compressRGBImageJPG(new Mat(cpuImage), yuvImage, compressedImage);
         return compressedImage.limit();
      }
   }

   private long openCVDepthCompression(Pointer cpuImage, Pointer gpuImage)
   {
      try (BytePointer compressedImage = new BytePointer())
      {
         OpenCVTools.compressImagePNG(new Mat(cpuImage), compressedImage);
         return compressedImage.limit();
      }
   }

   public static void main(String[] args) throws IOException
   {
      SVOImageCompressionAlgorithmComparison comparison = new SVOImageCompressionAlgorithmComparison();
      comparison.run();
      comparison.destroy();
   }
}
