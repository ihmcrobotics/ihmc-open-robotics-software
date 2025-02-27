package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Test that we can compute the average and keep a history.
 * Also ensures that things are printed correctly.
 */
public class FilteredRapidHeightMapExtractorTest
{
   @Test
   public void testGettingAverage()
   {
      int rows = 10;
      int cols = 10;
      int layers = 2;

      CUstream_st stream = CUDAStreamManager.getStream();
      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, rows, cols, layers);

      for (int i = 0; i < 8; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(i * 2 + 2));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat);
         Mat temp = new Mat();
         currentAverage.download(temp);
         PerceptionDebugTools.printMat("Current", temp, 1);
      }

      filteredRapidHeightMapExtractor.destroy();
   }

   @Test
   public void testChangeAfterSteadyState()
   {
      int rows = 2;
      int cols = 2;
      int layers = 2;

      CUstream_st stream = CUDAStreamManager.getStream();
      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, rows, cols, layers);

      // This example fills the history with the same value, all 8's
      // Lets see how this behaves when we later than introduce a change (noise)
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(800));
      for (int i = 0; i < 8; i++)
      {
         cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(800));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         filteredRapidHeightMapExtractor.update(latestDepthMat);
      }

      Mat cpuDataAdjusted = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(1000));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuDataAdjusted);

      GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat);
      Mat temp = new Mat();
      currentAverage.download(temp);
      PerceptionDebugTools.printMat("Current", temp, 1);

      for (int i = 0; i < temp.rows(); i++)
      {
         for (int j = 0; j < temp.cols(); j++)
         {
            // This data should be more than the latest data because it's weighted towards the previous average
            assertTrue(temp.col(i).row(j).data().getShort() > cpuData.col(j).data().getShort());
         }
      }

      filteredRapidHeightMapExtractor.destroy();
   }

   @Test
   public void testChangedAfterSteadyStateRealDepthValuesUnsignedShort()
   {
      int rows = 2;
      int cols = 2;
      int layers = 2;

      CUstream_st stream = CUDAStreamManager.getStream();
      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, rows, cols, layers);

      // This example fills the history with the same value, all 8's
      // Lets see how this behaves when we later than introduce a change (noise)
      for (int i = 0; i < 8; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(32768));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         filteredRapidHeightMapExtractor.update(latestDepthMat);
      }

      // 400 is about 20 centimeters? Ish depending on the parameters, this is hard coded could change it to be based on the parameters
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(33100));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuData);

      GpuMat currentAverage = filteredRapidHeightMapExtractor.update(latestDepthMat);
      Mat temp = new Mat();
      currentAverage.download(temp);
      PerceptionDebugTools.printMat("Current", temp, 1);

      for (int i = 0; i < temp.rows(); i++)
      {
         for (int j = 0; j < temp.cols(); j++)
         {
            // This data should be less than the latest data because it's weighted towards the previous average
            assertTrue(temp.col(i).row(j).data().getShort() < cpuData.col(j).data().getShort());
         }
      }

      filteredRapidHeightMapExtractor.destroy();
   }
}
