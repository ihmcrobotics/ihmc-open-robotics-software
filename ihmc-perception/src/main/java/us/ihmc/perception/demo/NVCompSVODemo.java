package us.ihmc.perception.demo;

import org.bytedeco.cuda.nvcomp.PimplManager;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO.RecordMode;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

import static org.bytedeco.cuda.global.cudart.cudaFreeHost;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

public class NVCompSVODemo extends NVCompDemo
{
   private static final String RESULT_FILE_DIRECTORY = System.getProperty("user.home") + File.separator + "Documents" + File.separator;
   private static final String SVO_FILE_NAME = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                        .toAbsolutePath()
                                                                                        .toString();
   private static final int MAX_DATA_POINTS = 100;

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "nvcomp_svo_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDColorDepthImageRetrieverSVO zedDataRetriever;

   private final Map<String, List<Double>> managerToCompressionRatioMapColor = new TreeMap<>();
   private final Map<String, List<Double>> managerToCompressionTimeMapColor = new TreeMap<>();
   private final Map<String, List<Double>> managerToDecompressionTimeMapColor = new TreeMap<>();

   private final Map<String, List<Double>> managerToCompressionRatioMapDepth = new TreeMap<>();
   private final Map<String, List<Double>> managerToCompressionTimeMapDepth = new TreeMap<>();
   private final Map<String, List<Double>> managerToDecompressionTimeMapDepth = new TreeMap<>();

   // FIXME: Using the CUDAImageEncoder messes up the nvcomp algorithms
//   private final CUDAImageEncoder cudaImageEncoder = new CUDAImageEncoder();
//   private final List<Double> cudaImageEncoderCompressionRatios = new ArrayList<>();
//   private final List<Double> cudaImageEncoderCompressionTimes = new ArrayList<>();
//   private final List<Double> cudaImageEncoderDecompressionTimes = new ArrayList<>();

   private final List<Double> opencvToolsColorCompressionRatios = new ArrayList<>();
   private final List<Double> opencvToolsColorCompressionTimes = new ArrayList<>();
   private final List<Double> opencvToolsColorDecompressionTimes = new ArrayList<>();

   private final List<Double> opencvToolsDepthCompressionRatios = new ArrayList<>();
   private final List<Double> opencvToolsDepthCompressionTimes = new ArrayList<>();
   private final List<Double> opencvToolsDepthDecompressionTimes = new ArrayList<>();


   protected NVCompSVODemo()
   {
      super();

      for (String managerName : compressionManagers.keySet())
      {
         managerToCompressionRatioMapColor.put(managerName, new ArrayList<>());
         managerToCompressionTimeMapColor.put(managerName, new ArrayList<>());
         managerToDecompressionTimeMapColor.put(managerName, new ArrayList<>());

         managerToCompressionRatioMapDepth.put(managerName, new ArrayList<>());
         managerToCompressionTimeMapDepth.put(managerName, new ArrayList<>());
         managerToDecompressionTimeMapDepth.put(managerName, new ArrayList<>());
      }
      zedDataRetriever = new ZEDColorDepthImageRetrieverSVO(0, () -> true, () -> true, ros2Helper, RecordMode.PLAYBACK, SVO_FILE_NAME);
      zedDataRetriever.grabOneFrame(); // initializes camera
   }

   @Override
   protected void runDemo()
   {
      int svoSize = zedDataRetriever.getLength();
      float frameSkip = (float) svoSize / MAX_DATA_POINTS;
      float nextFrame = 0.0f;
      for (int i = 0; i < MAX_DATA_POINTS; ++i)
      {
         int frameToGrab = Math.round(nextFrame);

         LogTools.info("Grabbing frame {}", frameToGrab);

         zedDataRetriever.setCurrentPosition(frameToGrab);
         zedDataRetriever.grabOneFrame();
         RawImage depthImage = zedDataRetriever.getLatestRawDepthImage();
         RawImage colorImage = zedDataRetriever.getLatestRawColorImage(RobotSide.LEFT);

         // Ensure images exist on CPU and GPU
         depthImage.getCpuImageMat();
         depthImage.getGpuImageMat();
         colorImage.getCpuImageMat();
         colorImage.getGpuImageMat();

         collectColorData(colorImage);
//         collectCUDAImageEncoderData(colorImage);
         collectOpenCVToolsColorData(colorImage);

         collectDepthData(depthImage);
         collectOpenCVToolsDepthData(depthImage);

         colorImage.release();
         depthImage.release();

         nextFrame += frameSkip;
      }

      try
      {
         String resultFileName = "NVCompSVODemoResults_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()) + ".csv";
         saveDataToFile(RESULT_FILE_DIRECTORY + resultFileName);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   protected void destroy()
   {
      super.destroy();

      ros2Node.destroy();
      zedDataRetriever.destroy();
   }

   private void collectColorData(RawImage rawImage)
   {
      Stopwatch stopwatch = new Stopwatch();
      Mat image = rawImage.getCpuImageMat();
      long imageSize = image.elemSize() * image.total();

      // NVCOMP Managers
      for (Entry<String, PimplManager> managerEntry : compressionManagers.entrySet())
      {
         BytePointer compressedImageData = new BytePointer();
         stopwatch.start();
         long compressedSize = compress(image, compressedImageData, managerEntry.getValue());
         double compressionTime = stopwatch.lap();
         Mat decompressedImage = new Mat(image.size(), image.type());
         decompressImage(compressedImageData, compressedSize, decompressedImage);
         double decompressionTime = stopwatch.lap();

         double compressionRatio = (double) imageSize / compressedSize;

         managerToCompressionRatioMapColor.get(managerEntry.getKey()).add(compressionRatio);
         managerToCompressionTimeMapColor.get(managerEntry.getKey()).add(compressionTime);
         managerToDecompressionTimeMapColor.get(managerEntry.getKey()).add(decompressionTime);

         checkCUDAError(cudaStreamSynchronize(stream));
         checkCUDAError(cudaFreeHost(compressedImageData));
         compressedImageData.close();
         decompressedImage.close();
      }
   }

   private void collectDepthData(RawImage rawImage)
   {
      Stopwatch stopwatch = new Stopwatch();
      Mat image = rawImage.getCpuImageMat();
      long imageSize = image.elemSize() * image.total();
      for (Entry<String, PimplManager> managerEntry : compressionManagers.entrySet())
      {
         stopwatch.start();
         BytePointer compressedImageData = new BytePointer();
         long compressedSize = compress(image, compressedImageData, managerEntry.getValue());
         double compressionTime = stopwatch.lap();
         Mat decompressedImage = new Mat(image.size(), image.type());
         decompressImage(compressedImageData, compressedSize, decompressedImage);
         double decompressionTime = stopwatch.lap();

         double compressionRatio = (double) imageSize / compressedSize;

         managerToCompressionRatioMapDepth.get(managerEntry.getKey()).add(compressionRatio);
         managerToCompressionTimeMapDepth.get(managerEntry.getKey()).add(compressionTime);
         managerToDecompressionTimeMapDepth.get(managerEntry.getKey()).add(decompressionTime);

         checkCUDAError(cudaStreamSynchronize(stream));
         checkCUDAError(cudaFreeHost(compressedImageData));
         decompressedImage.close();
      }
   }

//   private void collectCUDAImageEncoderData(RawImage rawImage)
//   {
//      Stopwatch stopwatch = new Stopwatch();
//      Mat image = rawImage.getCpuImageMat();
//      long imageSize = image.elemSize() * image.total();
//
//      BytePointer compressedImageData = new BytePointer(imageSize);
//      stopwatch.start();
//      cudaImageEncoder.encodeBGR(rawImage.getGpuImageMat().data(),
//                                 compressedImageData,
//                                 rawImage.getImageWidth(),
//                                 rawImage.getImageHeight(),
//                                 rawImage.getGpuImageMat().step());
//      double compressionTime = stopwatch.lap();
//      Mat compressedImage = new Mat(compressedImageData);
//      Mat decompressedImage = new Mat(image.size(), image.type());
//      opencv_imgcodecs.imdecode(compressedImage, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedImage);
//      double decompressionTime = stopwatch.lap();
//
//      double compressionRatio = (double) imageSize / compressedImageData.limit();
//
//      cudaImageEncoderCompressionRatios.add(compressionRatio);
//      cudaImageEncoderCompressionTimes.add(compressionTime);
//      cudaImageEncoderDecompressionTimes.add(decompressionTime);
//
//      compressedImageData.close();
//      compressedImage.close();
//      decompressedImage.close();
//   }

   private void collectOpenCVToolsColorData(RawImage rawImage)
   {
      Stopwatch stopwatch = new Stopwatch();
      Mat image = rawImage.getCpuImageMat();
      long imageSize = image.elemSize() * image.total();

      BytePointer compressedImageData = new BytePointer();
      stopwatch.start();
      OpenCVTools.compressImagePNG(image, compressedImageData);
      double compressionTime = stopwatch.lap();
      Mat compressedImage = new Mat(compressedImageData);
      Mat decompressedImage = new Mat(image.size(), image.type());
      opencv_imgcodecs.imdecode(compressedImage, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedImage);
      double decompressionTime = stopwatch.lap();

      double compressionRatio = (double) imageSize / compressedImageData.limit();

      opencvToolsColorCompressionRatios.add(compressionRatio);
      opencvToolsColorCompressionTimes.add(compressionTime);
      opencvToolsColorDecompressionTimes.add(decompressionTime);

      compressedImageData.close();
      compressedImage.close();
      decompressedImage.close();
   }

   private void collectOpenCVToolsDepthData(RawImage rawImage)
   {
      Stopwatch stopwatch = new Stopwatch();
      Mat image = rawImage.getCpuImageMat();
      long imageSize = image.elemSize() * image.total();

      BytePointer compressedImageData = new BytePointer();
      stopwatch.start();
      OpenCVTools.compressImagePNG(image, compressedImageData);
      double compressionTime = stopwatch.lap();
      Mat compressedImage = new Mat(compressedImageData);
      Mat decompressedImage = new Mat(image.size(), image.type());
      opencv_imgcodecs.imdecode(compressedImage, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedImage);
      double decompressionTime = stopwatch.lap();

      double compressionRatio = (double) imageSize / compressedImageData.limit();

      opencvToolsDepthCompressionRatios.add(compressionRatio);
      opencvToolsDepthCompressionTimes.add(compressionTime);
      opencvToolsDepthDecompressionTimes.add(decompressionTime);

      compressedImageData.close();
      compressedImage.close();
      decompressedImage.close();
   }

//   private void testDepthCompression(RawImage depthImage)
//   {
//      GpuMat gpuDepth = depthImage.getGpuImageMat();
//      GpuMat depthMSB = new GpuMat();
//      GpuMat depthLSB = new GpuMat();
//
//      GpuMat msbExtractor = new GpuMat(gpuDepth.size(), gpuDepth.type(), new Scalar(65280.0));
//      GpuMat lsbExtractor = new GpuMat(gpuDepth.size(), gpuDepth.type(), new Scalar(255.0));
//
//      opencv_cudaarithm.bitwise_and(gpuDepth, msbExtractor, depthMSB);
//      opencv_cudaarithm.bitwise_and(gpuDepth, lsbExtractor, depthLSB);
//
//      depthMSB.convertTo(depthMSB, opencv_core.CV_8UC1, 1.0 / 255.0, 0.0);
//      depthLSB.convertTo(depthLSB, opencv_core.CV_8UC1);
//   }

   private void saveDataToFile(String filePath) throws IOException
   {
      File file = new File(filePath);

      if (file.exists())
         file.delete();

      file.createNewFile();

      FileWriter writer = new FileWriter(filePath);
      writer.write("Compression Manager,Avg Color Compression Ratio,Avg Color Compression Time (S),Avg Color Decompression Time (S),Avg Depth Compression Ratio,Avg Depth Compression Time (S),Avg Depth Decompression Time (S)\n");
      for (String managerName : compressionManagers.keySet())
      {
         double averageColorCompressionRatio = getAverage(managerToCompressionRatioMapColor.get(managerName));
         double averageColorCompressionTime = getAverage(managerToCompressionTimeMapColor.get(managerName));
         double averageColorDecompressionTime = getAverage(managerToDecompressionTimeMapColor.get(managerName));

         double averageDepthCompressionRatio = getAverage(managerToCompressionRatioMapDepth.get(managerName));
         double averageDepthCompressionTime = getAverage(managerToCompressionTimeMapDepth.get(managerName));
         double averageDepthDecompressionTime = getAverage(managerToDecompressionTimeMapDepth.get(managerName));

         writer.write(String.format("%s,%f,%f,%f,%f,%f,%f\n",
                                    managerName,
                                    averageColorCompressionRatio,
                                    averageColorCompressionTime,
                                    averageColorDecompressionTime,
                                    averageDepthCompressionRatio,
                                    averageDepthCompressionTime,
                                    averageDepthDecompressionTime));
      }

      double opencvToolsAverageColorCompressionRatio = getAverage(opencvToolsColorCompressionRatios);
      double opencvToolsAverageColorCompressionTimes = getAverage(opencvToolsColorCompressionTimes);
      double opencvToolsAverageColorDecompressionTimes = getAverage(opencvToolsColorDecompressionTimes);

      double opencvToolsAverageDepthCompressionRatio = getAverage(opencvToolsDepthCompressionRatios);
      double opencvToolsAverageDepthCompressionTimes = getAverage(opencvToolsDepthCompressionTimes);
      double opencvToolsAverageDepthDecompressionTimes = getAverage(opencvToolsDepthDecompressionTimes);

      writer.write(String.format("%s,%f,%f,%f,%f,%f,%f\n",
                                 "OpenCVTools (PNG)",
                                 opencvToolsAverageColorCompressionRatio,
                                 opencvToolsAverageColorCompressionTimes,
                                 opencvToolsAverageColorDecompressionTimes,
                                 opencvToolsAverageDepthCompressionRatio,
                                 opencvToolsAverageDepthCompressionTimes,
                                 opencvToolsAverageDepthDecompressionTimes));

//      double averageCUDACompressionRatio = getAverage(cudaImageEncoderCompressionRatios);
//      double averageCUDACompressionTime = getAverage(cudaImageEncoderCompressionTimes);
//      double averageCUDADecompressionTime = getAverage(cudaImageEncoderDecompressionTimes);
//
//      writer.write(String.format("%s,%f,%f,%f,,,\n",
//                   "CUDAImageEncoder",
//                   averageCUDACompressionRatio,
//                   averageCUDACompressionTime,
//                   averageCUDADecompressionTime));

      writer.close();
   }

   private double getAverage(List<Double> doubleList)
   {
      return doubleList.stream().reduce(Double::sum).get() / doubleList.size();
   }

   public static void main(String[] args)
   {
      NVCompSVODemo demo = new NVCompSVODemo();
      demo.runDemo();
      demo.destroy();
   }
}
