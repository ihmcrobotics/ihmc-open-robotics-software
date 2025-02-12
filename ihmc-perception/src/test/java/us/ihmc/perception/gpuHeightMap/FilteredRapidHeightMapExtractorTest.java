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
      int rows = 1501;
      int cols = 1501;
      CUstream_st stream = CUDAStreamManager.getStream();
      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, rows, cols);

      for (int i = 0; i < 8; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(i * 2 + 2));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
         Mat temp = new Mat();
         currentAverage.download(temp);
//         PerceptionDebugTools.printMat("current", temp, 1);
      }

      filteredRapidHeightMapExtractor.destroy();
   }

   @Test
   public void testChangeAfterSteadyState()
   {
      int rows = 2;
      int cols = 2;
      CUstream_st stream = CUDAStreamManager.getStream();
      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, rows, cols);

      // This example fills the history with the same value, all 8's
      // Lets see how this behaves when we later then introduce a change (noise)
      for (int i = 0; i < 8; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(800));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      }

      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(1000));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuData);

      GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      Mat temp = new Mat();
      currentAverage.download(temp);
      PerceptionDebugTools.printMat("current", temp, 1);

      filteredRapidHeightMapExtractor.destroy();
   }

   @Test
   public void testChangedAfterSteadyStateRealDepthValuesUnsignedShort()
   {
      int rows = 2;
      int cols = 2;
      CUstream_st stream = CUDAStreamManager.getStream();
      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, rows, cols);

      // This example fills the history with the same value, all 8's
      // Lets see how this behaves when we later then introduce a change (noise)
      for (int i = 0; i < 8; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(32768));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      }

      // 400 is about 20 centimeters? Ish depending on the parameters, this is hard coded could change it to be based on the parameters
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(33100));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuData);

      GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      Mat temp = new Mat();
      currentAverage.download(temp);
      PerceptionDebugTools.printMat("current", temp, 1);

      filteredRapidHeightMapExtractor.destroy();
   }


   // TODO remove this guy heheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
   @Test
   public void testDefaultValueSkipping()
   {
      int rows = 3;
      int cols = 3;
      CUstream_st stream = CUDAStreamManager.getStream();
      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, rows, cols);

      // Set everything to 1, I'm gonna pass this in as the default value so these get skipped hopefully
      for (int i = 0; i < 4; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(1));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      }

      //TODO maybe we can just skip this? Does it matter
      // I guess the newest height map will have the same values, so we can skip it

      // 400 is about 20 centimeters? Ish depending on the parameters, this is hard coded could change it to be based on the parameters
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(33100));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuData);

      GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      Mat temp = new Mat();
      currentAverage.download(temp);
      PerceptionDebugTools.printMat("current", temp, 1);

      filteredRapidHeightMapExtractor.destroy();
   }
}
