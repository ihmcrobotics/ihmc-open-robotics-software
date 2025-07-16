package us.ihmc.perception.demo;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencv.OpenCVTools;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.function.BiFunction;

public class ImageCompressionAlgorithmComparison
{
   private static final String RESULT_FILE_DIRECTORY = System.getProperty("user.home") + File.separator + "Documents" + File.separator;

   private final SortedMap<String, CompressionAlgorithmAnalyzerPair> algorithmAnalyzers = new TreeMap<>();

   public void addColorCompressionAlgorithm(String algorithmName, BiFunction<Pointer, Pointer, Long> compressionFunction)
   {
      algorithmAnalyzers.putIfAbsent(algorithmName, new CompressionAlgorithmAnalyzerPair());
      algorithmAnalyzers.get(algorithmName).setColorAnalyzer(new CompressionAlgorithmAnalyzer(compressionFunction));
   }

   public void addDepthCompressionAlgorithm(String algorithmName, BiFunction<Pointer, Pointer, Long> compressionFunction)
   {
      algorithmAnalyzers.putIfAbsent(algorithmName, new CompressionAlgorithmAnalyzerPair());
      algorithmAnalyzers.get(algorithmName).setDepthAnalyzer(new CompressionAlgorithmAnalyzer(compressionFunction));
   }

   public void test(RawImage image)
   {
      long dataSize = OpenCVTools.dataSize(image.getCpuImageMat());

      for (CompressionAlgorithmAnalyzerPair analyzerPair : algorithmAnalyzers.values())
      {
         if (image.getOpenCVType() == opencv_core.CV_16U)
         {
            if (analyzerPair.hasDepthAnalyzer())
               analyzerPair.getDepthAnalyzer().testCompression(image.getCpuImageMat(), image.getGpuImageMat(), dataSize);
         }
         else
         {
            if (analyzerPair.hasColorAnalyzer())
               analyzerPair.getColorAnalyzer().testCompression(image.getCpuImageMat(), image.getGpuImageMat(), dataSize);
         }
      }
   }

   public void saveResultToFile(String fileName) throws IOException
   {
      String filePath = RESULT_FILE_DIRECTORY + fileName;
      File file = new File(filePath);
      if (file.exists())
         if (!file.delete())
            LogTools.fatal("Failed to delete existing file: {}", filePath);

      if (!file.createNewFile())
         LogTools.fatal("Failed to create a new file: {}", filePath);

      try (FileWriter writer = new FileWriter(filePath))
      {
         // Write the header
         writer.write("Compression Algorithm,Avg Color Compressed Size (bytes),Avg Color Compression Ratio,Avg Color Compression Duration (S),Avg Depth Compressed Size (bytes),Avg Depth Compression Ratio,Avg Depth Compression Duration (S)\n");

         for (Map.Entry<String, CompressionAlgorithmAnalyzerPair> algorithmAnalyzerEntry : algorithmAnalyzers.entrySet())
         {
            CompressionAlgorithmAnalyzer colorAnalyzer = algorithmAnalyzerEntry.getValue().getColorAnalyzer();
            CompressionAlgorithmAnalyzer depthAnalyzer = algorithmAnalyzerEntry.getValue().getDepthAnalyzer();
            writer.write(String.format("%s,%s,%s,%s,%s,%s,%s\n",
                                       algorithmAnalyzerEntry.getKey(),
                                       colorAnalyzer == null ? "" : format(colorAnalyzer.averageCompressedDataSize()),
                                       colorAnalyzer == null ? "" : format(colorAnalyzer.averageCompressionRatio()),
                                       colorAnalyzer == null ? "" : format(colorAnalyzer.averageCompressionDuration()),
                                       depthAnalyzer == null ? "" : format(depthAnalyzer.averageCompressedDataSize()),
                                       depthAnalyzer == null ? "" : format(depthAnalyzer.averageCompressionRatio()),
                                       depthAnalyzer == null ? "" : format(depthAnalyzer.averageCompressionDuration())));
         }
      }
   }

   private String format(double value)
   {
      return String.format("%.6f", value);
   }

   private static class CompressionAlgorithmAnalyzerPair
   {
      private CompressionAlgorithmAnalyzer colorAnalyzer = null;
      private CompressionAlgorithmAnalyzer depthAnalyzer = null;

      private boolean hasColorAnalyzer()
      {
         return colorAnalyzer != null;
      }

      private boolean hasDepthAnalyzer()
      {
         return depthAnalyzer != null;
      }

      private CompressionAlgorithmAnalyzer getColorAnalyzer()
      {
         return colorAnalyzer;
      }

      private CompressionAlgorithmAnalyzer getDepthAnalyzer()
      {
         return depthAnalyzer;
      }

      private void setColorAnalyzer(CompressionAlgorithmAnalyzer colorAnalyzer)
      {
         this.colorAnalyzer = colorAnalyzer;
      }

      private void setDepthAnalyzer(CompressionAlgorithmAnalyzer depthAnalyzer)
      {
         this.depthAnalyzer = depthAnalyzer;
      }
   }
}
