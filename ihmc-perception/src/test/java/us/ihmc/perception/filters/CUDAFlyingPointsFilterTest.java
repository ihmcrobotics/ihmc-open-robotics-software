package us.ihmc.perception.filters;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.junit.jupiter.api.Assertions.*;

public class CUDAFlyingPointsFilterTest
{
   // Tests a simple 3x3 matrix where the input contains an outlier.
   // The test validates whether the kernel correctly replaces the outlier with the median of the surrounding values.
   @Test
   public void testSimpleMatrix() throws Exception
   {
      CUDAFlyingPointsFilter flyingPointsFilter;
      Mat filterGpuMat;
      Mat outputMat = new Mat();
      flyingPointsFilter = new CUDAFlyingPointsFilter();

      Mat inputMat = new Mat(3, 3, opencv_core.CV_16UC1);
      inputMat.ptr(0, 0).putShort((short) 10);
      inputMat.ptr(0, 1).putShort((short) 10);
      inputMat.ptr(0, 2).putShort((short) 10);
      inputMat.ptr(1, 0).putShort((short) 10);
      inputMat.ptr(1, 1).putShort((short) 15); // outlier
      inputMat.ptr(1, 2).putShort((short) 10);
      inputMat.ptr(2, 0).putShort((short) 10);
      inputMat.ptr(2, 1).putShort((short) 10);
      inputMat.ptr(2, 2).putShort((short) 10);

      PerceptionDebugTools.printMat("input_matrix", inputMat, 1);

      filterGpuMat = flyingPointsFilter.applyFilter(inputMat);
      filterGpuMat.copyTo(outputMat);

      PerceptionDebugTools.printMat("output_matrix", outputMat, 1);

      // Validate that all elements in outputMat are 10
      for (int i = 0; i < outputMat.rows(); i++)
      {
         for (int j = 0; j < outputMat.cols(); j++)
         {
            assertEquals(10, outputMat.ptr(i, j).get(), "Element [" + i + "][" + j + "]");
         }
      }
      filterGpuMat.close();
      outputMat.close();
      flyingPointsFilter.destroy();
   }
}