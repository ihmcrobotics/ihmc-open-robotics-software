package us.ihmc.perception.filters;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;

public class CUDAFlyingPointsFilterTest
{
   private CUDAFlyingPointsFilter flyingPointsFilter;
   private GpuMat inputGpuMat = new GpuMat();
   private GpuMat outputGpuMat = new GpuMat();
   private Mat outputMat = new Mat();

   private static void printResult(Mat cpuResult)
   {
      for (int i = 0; i < cpuResult.rows(); ++i)
      {
         for (int j = 0; j < cpuResult.cols(); ++j)
         {
            // Don't want to make it to easy to get the data though :)
            System.out.print(cpuResult.row(i).col(j).data().getShort() + " ");
         }

         System.out.println();
      }
   }

   @Test
   public void testSimpleMatrix()
   {
      flyingPointsFilter = new CUDAFlyingPointsFilter();
      Mat inputMat = new Mat(4, 4, opencv_core.CV_8UC3, new Scalar(1));
      inputGpuMat.upload(inputMat);
      outputGpuMat = flyingPointsFilter.applyFilter(inputGpuMat);
      outputGpuMat.download(outputMat);
      printResult(outputMat);
   }
}
