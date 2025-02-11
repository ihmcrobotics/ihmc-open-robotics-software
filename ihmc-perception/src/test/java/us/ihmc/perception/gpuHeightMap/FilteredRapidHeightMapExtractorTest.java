package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.tools.PerceptionDebugTools;

public class FilteredRapidHeightMapExtractorTest
{
   @Test
   public void testGettingAverage()
   {
      int rows = 2;
      int cols = 2;
      CUstream_st stream = CUDAStreamManager.getStream();
      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, rows, cols);

      for (int i = 0; i < 8; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(i * 2 + 2));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat);
         Mat temp = new Mat();
         currentAverage.download(temp);
         PerceptionDebugTools.printMat("current", temp, 1);
      }

      filteredRapidHeightMapExtractor.close();
   }
}
