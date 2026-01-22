package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.jcodec.common.Assert;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class HeightMapUtilsTest
{
   private IntPointer indexPointer;
   private FloatPointer centerPointer;
   private FloatPointer resolutionPointer;
   private IntPointer centerIndexPointer;
   private FloatPointer xHostPointer;
   private FloatPointer yHostPointer;

   private FloatPointer coordinatePointer;

   @BeforeEach
   public void setupPointers()
   {
      setupPointersForIndicesToCoordinateKernel();
      setupPointersForCoordinateToIndicesKernel();
   }

   private void setupPointersForIndicesToCoordinateKernel()
   {
      indexPointer = new IntPointer(1);
      indexPointer.put(0, 5);

      centerPointer = new FloatPointer(1);
      centerPointer.put(0, 0.0f);

      resolutionPointer = new FloatPointer(1);
      resolutionPointer.put(0, 1.0f);

      centerIndexPointer = new IntPointer(1);
      centerIndexPointer.put(0, 5);

      xHostPointer = new FloatPointer(1);
      yHostPointer = new FloatPointer(1);
   }

   private void setupPointersForCoordinateToIndicesKernel()
   {
      coordinatePointer = new FloatPointer(1);
      coordinatePointer.put(0, 1.0f);
   }

   @AfterEach
   public void closePointers()
   {
      indexPointer.close();
      centerPointer.close();
      resolutionPointer.close();
      centerIndexPointer.close();
      xHostPointer.close();
      yHostPointer.close();

      coordinatePointer.close();
   }

   /**
    * Test to ensure that the CUDA kernel is working when using the HeightMapUtils.cuh header file.
    * The method {@link HeightMapUtilsTest#runIndicesToCoordinateOnCUDA()} returns the result from the GPU so we can compare it with the expected values.
    */
   @Test
   public void testIndicesToCoordinateCUDA() throws Exception
   {
      float[] result = runIndicesToCoordinateOnCUDA();

      LogTools.info("xResult: " + result[0]);
      LogTools.info("yResult: " + result[1]);

      // These expected values are pulled from the HeightMapUtils.cuh file with the parameters used in this test
      // It should match the OpenCL version, use that to verify the equations
      int expectedXValueFromKernel = 0;
      int expectedYValueFromKernel = -4;

      Assert.assertEquals(expectedXValueFromKernel, (int) result[0]);
      Assert.assertEquals(expectedYValueFromKernel, (int) result[1]);
   }

   @Test
   public void testCoordinateToIndicesCUDA() throws Exception
   {
      float[] result = runCoordinateToIndicesOnCUDA();

      LogTools.info("xResult: " + result[0]);
      LogTools.info("yResult: " + result[1]);

      // These expected values are pulled from the HeightMapUtils.cuh file with the parameters used in this test
      // It should match the OpenCL version, use that to verify the equations
      // round((coordinate - center) / resolution) + center_index)
      int expectedXValueFromKernel = Math.round((coordinatePointer.get(0) - centerPointer.get(0)) / 5) + centerIndexPointer.get(0);
      int expectedYValueFromKernel = Math.round((coordinatePointer.get(0) - centerPointer.get(0)) / 5) + centerIndexPointer.get(0);

      Assert.assertEquals("Expected value: " + expectedXValueFromKernel + " and actual value: " + result[0], expectedXValueFromKernel, (int) result[0]);
      Assert.assertEquals("Expected value: " + expectedXValueFromKernel + " and actual value: " + result[0], expectedYValueFromKernel, (int) result[1]);
   }

   private float[] runIndicesToCoordinateOnCUDA() throws Exception
   {
      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/HeightMapUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/HeightMapUtils.cuh");

      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(programPath, headerPath);
      CUDAKernel kernel = program.loadKernel("test_indices_to_coordinate");

      // These will be pointers to the gpu memory, where we will upload the data too.
      FloatPointer gpuCenterPointer = new FloatPointer();
      FloatPointer gpuResolutionPointer = new FloatPointer();
      FloatPointer gpuXHostPointer = new FloatPointer();
      FloatPointer gpuYHostPointer = new FloatPointer();

      long centerPointerSize = centerPointer.limit() + 1;
      long resolutionPointerSize = resolutionPointer.limit() + 1;
      long xHostPointerSize = xHostPointer.limit() + 1;
      long yHostPointerSize = yHostPointer.limit() + 1;

      CUDATools.mallocAsync(gpuCenterPointer, centerPointerSize, stream);
      CUDATools.mallocAsync(gpuResolutionPointer, resolutionPointerSize, stream);
      CUDATools.mallocAsync(gpuXHostPointer, xHostPointerSize, stream);
      CUDATools.mallocAsync(gpuYHostPointer, yHostPointerSize, stream);

      CUDATools.memcpyAsync(gpuCenterPointer, centerPointer, centerPointerSize, stream);
      CUDATools.memcpyAsync(gpuResolutionPointer, resolutionPointer, resolutionPointerSize, stream);
      CUDATools.memcpyAsync(gpuXHostPointer, xHostPointer, xHostPointerSize, stream);
      CUDATools.memcpyAsync(gpuYHostPointer, yHostPointer, yHostPointerSize, stream);

      kernel.withInt(5).withPointer(gpuCenterPointer).withPointer(gpuResolutionPointer);
      kernel.withInt(5).withPointer(gpuXHostPointer).withPointer(gpuYHostPointer);

      // This is where the kernel is run, for more help with CUDA Examples look at the ExampleCUDAKernel class
      kernel.run(stream, new dim3(), new dim3(), 0);

      CUDATools.memcpyAsync(xHostPointer, gpuXHostPointer, xHostPointerSize, stream);
      CUDATools.memcpyAsync(yHostPointer, gpuYHostPointer, yHostPointerSize, stream);

      cudaStreamSynchronize(stream);

      // Get the result from the GPU and store in on the CPU (xResult and yResult)
      float[] xResult = new float[1];
      float[] yResult = new float[1];
      xHostPointer.get(xResult);
      yHostPointer.get(yResult);

      cudaFreeAsync(gpuCenterPointer, stream);
      cudaFreeAsync(gpuCenterPointer, stream);
      cudaFreeAsync(gpuResolutionPointer, stream);
      cudaFreeAsync(gpuXHostPointer, stream);
      cudaFreeAsync(gpuYHostPointer, stream);

      kernel.close();
      program.close();

      gpuCenterPointer.close();
      gpuResolutionPointer.close();
      gpuXHostPointer.close();
      gpuYHostPointer.close();

      CUDAStreamManager.releaseStream(stream);

      return new float[] {xResult[0], yResult[0]};
   }

   private float[] runCoordinateToIndicesOnCUDA() throws Exception
   {
      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/HeightMapUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/HeightMapUtils.cuh");

      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(programPath, headerPath);
      CUDAKernel kernel = program.loadKernel("test_coordinate_to_indices");

      // These will be pointers to the gpu memory, where we will upload the data too.
      FloatPointer gpuCoordinatePointer = new FloatPointer();
      FloatPointer gpuCenterPointer = new FloatPointer();
      FloatPointer gpuXHostPointer = new FloatPointer();
      FloatPointer gpuYHostPointer = new FloatPointer();

      long centerPointerSize = centerPointer.limit() + 1;
      long resolutionPointerSize = resolutionPointer.limit() + 1;
      long xHostPointerSize = xHostPointer.limit() + 1;
      long yHostPointerSize = yHostPointer.limit() + 1;

      CUDATools.mallocAsync(gpuCoordinatePointer, centerPointerSize, stream);
      CUDATools.mallocAsync(gpuCenterPointer, resolutionPointerSize, stream);
      CUDATools.mallocAsync(gpuXHostPointer, xHostPointerSize, stream);
      CUDATools.mallocAsync(gpuYHostPointer, yHostPointerSize, stream);

      CUDATools.memcpyAsync(gpuCoordinatePointer, coordinatePointer, centerPointerSize, stream);
      CUDATools.memcpyAsync(gpuCenterPointer, centerPointer, centerPointerSize, stream);
      CUDATools.memcpyAsync(gpuXHostPointer, xHostPointer, xHostPointerSize, stream);
      CUDATools.memcpyAsync(gpuYHostPointer, yHostPointer, yHostPointerSize, stream);

      kernel.withPointer(gpuCoordinatePointer).withPointer(gpuCenterPointer);
      kernel.withFloat(5.0f).withInt(5);
      kernel.withPointer(gpuXHostPointer).withPointer(gpuYHostPointer);

      // This is where the kernel is run, for more help with CUDA Examples look at the ExampleCUDAKernel class
      kernel.run(stream, new dim3(), new dim3(), 0);

      CUDATools.memcpyAsync(xHostPointer, gpuXHostPointer, xHostPointerSize, stream);
      CUDATools.memcpyAsync(yHostPointer, gpuYHostPointer, yHostPointerSize, stream);

      cudaStreamSynchronize(stream);

      // Get the result from the GPU and store in on the CPU (xResult and yResult)
      float[] xResult = new float[1];
      float[] yResult = new float[1];
      xHostPointer.get(xResult);
      yHostPointer.get(yResult);

      cudaFreeAsync(gpuCoordinatePointer, stream);
      cudaFreeAsync(gpuCoordinatePointer, stream);
      cudaFreeAsync(gpuCenterPointer, stream);
      cudaFreeAsync(gpuXHostPointer, stream);
      cudaFreeAsync(gpuYHostPointer, stream);

      kernel.close();
      program.close();

      gpuCoordinatePointer.close();
      gpuCenterPointer.close();
      gpuXHostPointer.close();
      gpuYHostPointer.close();

      CUDAStreamManager.releaseStream(stream);

      return new float[] {xResult[0], yResult[0]};
   }
}