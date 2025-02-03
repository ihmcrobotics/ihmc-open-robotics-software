package us.ihmc.perception.filters;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;

public class CUDAFlyingPointsFilterTest
{
   private CUDAFlyingPointsFilter flyingPointsFilter;
   private GpuMat inputGpuMat = new GpuMat();
   private GpuMat outputGpuMat;
   private Mat outputMat = new Mat();

   /**
    * Utility method to print the matrix values for debugging.
    *
    * @param cpuResult The matrix to be printed.
    */
   private static void printResult(Mat cpuResult)
   {
      for (int i = 0; i < cpuResult.rows(); ++i)
      {
         for (int j = 0; j < cpuResult.cols(); ++j)
         {
            // Extracting and printing matrix values
            System.out.print(cpuResult.row(i).col(j).data().getShort() + " ");
         }
         System.out.println();
      }
   }

   /**
    * Test case for validating the CUDA flying points filter with a simple 3x3 matrix.
    */
   @Test
   public void testSimpleMatrix() throws Exception
   {
      flyingPointsFilter = new CUDAFlyingPointsFilter();

      // Creating a 3x3 matrix with 16-bit unsigned integer values
      Mat inputMat = new Mat(3, 3, opencv_core.CV_16UC1);

      // Populating the matrix with test values
      inputMat.ptr(0, 0).putShort((short) 1);
      inputMat.ptr(0, 1).putShort((short) 1);
      inputMat.ptr(0, 2).putShort((short) 1);
      inputMat.ptr(1, 0).putShort((short) 1);
      inputMat.ptr(1, 1).putShort((short) 3);
      inputMat.ptr(1, 2).putShort((short) 1);
      inputMat.ptr(2, 0).putShort((short) 1);
      inputMat.ptr(2, 1).putShort((short) 1);
      inputMat.ptr(2, 2).putShort((short) 1);

      // Printing the input matrix before processing
      printResult(inputMat);

      // Uploading input matrix to GPU memory
      inputGpuMat.upload(inputMat);

      // Applying the CUDA filter
      outputGpuMat = flyingPointsFilter.applyFilter(inputMat);

      // Downloading processed matrix back to CPU memory
      outputGpuMat.download(outputMat);

      // Printing the filtered output matrix
      printResult(outputMat);
   }
}

