package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;
import static org.junit.jupiter.api.Assertions.*;

public class RapidHeightMapExtractorCUDATest
{
   private static final int BLOCK_SIZE_XY = 32;

   private CUDAProgram heightMapCUDAProgram;

   /**
    * We need to account for drift in the height map.
    * This test tests the kernel to set the offset and move the height map up or down based on some value.
    */
   @Test
   public void testDriftInZOffsetKernel()
   {
      // Add this offset to each cell in the Mat
      float offsetToAdjustBy = 0.1f;
      int error;

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("MathUtils.cuh");
      URL kernelPath = getClass().getResource("RapidHeightMapExtractor.cu");

      int cellsPerAxis = 10;
      GpuMat matImage = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      GpuMat emptyMatImage = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      // Fill the mat with 1's
      matImage.setTo(new Scalar(1));
      emptyMatImage.setTo(new Scalar(0));
      CUstream_st stream = CUDAStreamManager.getStream();

      try
      {
         // We are trying to test the offset kernel to make sure the height map can adjust to drift in the z direction properly
         heightMapCUDAProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);
         CUDAKernel planOffsetKernel = heightMapCUDAProgram.loadKernel("planOffsetKernel");

         dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
         int gridSizeXY = (matImage.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 gridDim = new dim3(gridSizeXY, gridSizeXY, 1);

         // Run the snapping kernel
         planOffsetKernel.withPointer(matImage.data()).withLong(matImage.step());
         planOffsetKernel.withPointer(emptyMatImage.data()).withLong(emptyMatImage.step());
         planOffsetKernel.withFloat(offsetToAdjustBy).withInt(matImage.rows()).withInt(matImage.cols());

         planOffsetKernel.run(stream, gridDim, blockSize, 0);
         error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      Mat cpuMat = new Mat();
      matImage.download(cpuMat);

      // The expected value should be scaled by 10000
      float expectedValue = (float) ((float) 1.0 + (0.1 * 10000));
      short shortExpectedValue = (short) expectedValue;

      for (int i = 0; i < cpuMat.rows(); i++)
      {
         for (int j = 0; j < cpuMat.cols(); j++)
         {
            assertEquals(cpuMat.ptr(i,j).getShort(), shortExpectedValue);
         }
      }

      PerceptionDebugTools.printMat("Mat with Offset", cpuMat, 1);

      matImage.close();

      heightMapCUDAProgram.close();

      // At the end we have to destroy the stream to release the memory
      CUDAStreamManager.releaseStream(stream);
   }
}
