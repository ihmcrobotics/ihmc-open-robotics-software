package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.jcodec.common.Assert;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.opencl.OpenCLManager;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.cudart.cudaMemcpyAsync;

@Tag("ihmc-perception")
public class HeightMapUtilsTest
{
   private IntPointer indexPointer;
   private FloatPointer centerPointer;
   private FloatPointer resolutionPointer;
   private IntPointer centerIndexPointer;
   private FloatPointer xHostPointer;
   private FloatPointer yHostPointer;

   private FloatPointer coordinatePointer;
   private final int intBytes = Integer.BYTES;
   private final int floatBytes = Float.BYTES;

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
    * The method {@link HeightMapUtilsTest#runIndexToCoordinateOnOpenCL()} returns the result from the GPU so we can compare it with the expected values.
    */
   @Test
   public void testIndexToCoordinateOpenCL()
   {
      float[] result = runIndexToCoordinateOnOpenCL();
      Assert.assertNotNull(result);

      float expectedXValueFromKernel = (indexPointer.get(0) - centerIndexPointer.get()) * resolutionPointer.get(0) + centerPointer.get(0);
      int yIndexValue = 1;// Represents the y value in indexForConversion
      float expectedYValueFromKernel = (yIndexValue - centerIndexPointer.get()) * resolutionPointer.get(0) + centerPointer.get(0);

      Assert.assertEquals((int) expectedXValueFromKernel, (int) result[0]);
      Assert.assertEquals((int) expectedYValueFromKernel, (int) result[1]);

      LogTools.info(result[0] + " is the X kernel result!");
      LogTools.info(expectedXValueFromKernel + " is the X expected value!");
      LogTools.info(result[1] + " is the Y kernel result!");
      LogTools.info(expectedYValueFromKernel + " is the Y expected value!");
   }

   /**
    * Test to ensure that the CUDA kernel is working when using the HeightMapUtils.cuh header file.
    * The method {@link HeightMapUtilsTest#runIndicesToCoordinateOnCUDA()} returns the result from the GPU so we can compare it with the expected values.
    */
   @Test
   public void testIndicesToCoordinateCUDA()
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

   /**
    * We use the same methods in both OpenCL and in CUDA, these methods need to return the same result.
    * This test ensures that the results that are returned
    * from the GPU are the same for each kernel.
    */
   @Test
   public void testOpenCLMatchesCUDAResultForIndicesToCoordinate()
   {
      float[] openCLResult = runIndexToCoordinateOnOpenCL();
      float[] cudaResult = runIndicesToCoordinateOnCUDA();

      Assert.assertEquals(
            "The results don't match!\n" + "The openCL result (" + openCLResult[0] + ") is expected to match the cuda result (" + cudaResult[0] + ")",
            (int) openCLResult[0],
            (int) cudaResult[0]);
      Assert.assertEquals(
            "The results don't match!\n" + "The openCL result (" + openCLResult[0] + ") is expected to match the cuda result (" + cudaResult[0] + ")",
            (int) openCLResult[1],
            (int) cudaResult[1]);
   }

   @Test
   public void testCoordinateToIndicesOpenCL()
   {
      float[] result = runCoordinateToIndicesOnOpenCL();

      LogTools.info("xResult: " + result[0]);
      LogTools.info("yResult: " + result[1]);

      // These expected values are pulled from the HeightMapUtils.cuh file with the parameters used in this test
      // It should match the OpenCL version, use that to verify the equations
      // round((coordinate - center) / resolution) + center_index)
      int expectedXValueFromKernel = Math.round((coordinatePointer.get(0) - centerPointer.get(0)) / 5) + centerIndexPointer.get(0);
      int expectedYValueFromKernel = Math.round((coordinatePointer.get(0) - centerPointer.get(0)) / 5) + centerIndexPointer.get(0);

      Assert.assertEquals("Expected value: " + expectedXValueFromKernel + " and actual value: " + result[0], expectedXValueFromKernel, (int) result[0]);
      Assert.assertEquals("Expected value: " + expectedXValueFromKernel + " and actual value: " + result[0], expectedYValueFromKernel, (int) result[1]);   }

   @Test
   public void testCoordinateToIndicesCUDA()
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

   /**
    * We use the same methods in both OpenCL and in CUDA, these methods need to return the same result.
    * This test ensures that the results that are returned
    * from the GPU are the same for each kernel.
    */
   @Test
   public void testOpenCLMatchesCUDAResultForCoordinateToIndices()
   {
      float[] openCLResult = runCoordinateToIndicesOnOpenCL();
      float[] cudaResult = runCoordinateToIndicesOnCUDA();

      Assert.assertEquals(
            "The results don't match!\n" + "The openCL result (" + openCLResult[0] + ") is expected to match the cuda result (" + cudaResult[0] + ")",
            (int) openCLResult[0],
            (int) cudaResult[0]);
      Assert.assertEquals(
            "The results don't match!\n" + "The openCL result (" + openCLResult[0] + ") is expected to match the cuda result (" + cudaResult[0] + ")",
            (int) openCLResult[1],
            (int) cudaResult[1]);
   }

   private float[] runIndexToCoordinateOnOpenCL()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      _cl_program openCLProgram = openCLManager.loadProgram("HeightMapUtilsTest", "HeightMapUtils.cl");
      _cl_kernel openCLKernel = openCLManager.createKernel(openCLProgram, "test_indices_to_coordinate");

      // This creates the (global int* index) parameter used in the kernel.
      // Then this adds the data inside indexPointer to be the first argument for the kernel
      _cl_mem indexPointerObject = openCLManager.createBufferObject(intBytes, indexPointer);
      openCLManager.enqueueWriteBuffer(indexPointerObject, intBytes, indexPointer);
      openCLManager.setKernelArgument(openCLKernel, 0, indexPointerObject);

      // We need to create the 6 parameters used in the kernel.
      // The following sequence is repeated to create all the parameters correctly.
      _cl_mem centerPointerObject = openCLManager.createBufferObject(floatBytes, centerPointer);
      openCLManager.enqueueWriteBuffer(centerPointerObject, floatBytes, centerPointer);
      openCLManager.setKernelArgument(openCLKernel, 1, centerPointerObject);

      _cl_mem resolutionPointerObject = openCLManager.createBufferObject(floatBytes, resolutionPointer);
      openCLManager.enqueueWriteBuffer(resolutionPointerObject, floatBytes, resolutionPointer);
      openCLManager.setKernelArgument(openCLKernel, 2, resolutionPointerObject);

      _cl_mem centerIndexPointerObject = openCLManager.createBufferObject(intBytes, centerIndexPointer);
      openCLManager.enqueueWriteBuffer(centerIndexPointerObject, intBytes, centerIndexPointer);
      openCLManager.setKernelArgument(openCLKernel, 3, centerIndexPointerObject);

      _cl_mem xHostPointerObject = openCLManager.createBufferObject(floatBytes, xHostPointer);
      openCLManager.enqueueWriteBuffer(xHostPointerObject, floatBytes, xHostPointer);
      openCLManager.setKernelArgument(openCLKernel, 4, xHostPointerObject);

      _cl_mem yHostPointerObject = openCLManager.createBufferObject(floatBytes, yHostPointer);
      openCLManager.enqueueWriteBuffer(yHostPointerObject, floatBytes, yHostPointer);
      openCLManager.setKernelArgument(openCLKernel, 5, yHostPointerObject);

      openCLManager.execute1D(openCLKernel, 1);

      openCLManager.enqueueReadBuffer(xHostPointerObject, floatBytes, xHostPointer);
      openCLManager.enqueueReadBuffer(yHostPointerObject, floatBytes, yHostPointer);

      openCLProgram.close();
      openCLManager.destroy();

      return new float[] {xHostPointer.get(0), yHostPointer.get(0)};
   }

   private float[] runIndicesToCoordinateOnCUDA()
   {
      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuHeightMap/HeightMapUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuHeightMap/HeightMapUtils.cuh");

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

      cudaMallocAsync(gpuCenterPointer, gpuCenterPointer.sizeof() * centerPointerSize, stream);
      cudaMallocAsync(gpuResolutionPointer, gpuResolutionPointer.sizeof() * resolutionPointerSize, stream);
      cudaMallocAsync(gpuXHostPointer, gpuXHostPointer.sizeof() * xHostPointerSize, stream);
      cudaMallocAsync(gpuYHostPointer, gpuYHostPointer.sizeof() * yHostPointerSize, stream);

      cudaMemcpyAsync(gpuCenterPointer, centerPointer, (long) centerPointer.sizeof() * centerPointerSize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(gpuResolutionPointer, resolutionPointer, (long) resolutionPointer.sizeof() * resolutionPointerSize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(gpuXHostPointer, xHostPointer, (long) xHostPointer.sizeof() * xHostPointerSize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(gpuYHostPointer, yHostPointer, (long) yHostPointer.sizeof() * yHostPointerSize, cudaMemcpyDefault, stream);

      kernel.withInt(5).withPointer(gpuCenterPointer).withPointer(gpuResolutionPointer);
      kernel.withInt(5).withPointer(gpuXHostPointer).withPointer(gpuYHostPointer);

      // This is where the kernel is run, for more help with CUDA Examples look at the ExampleCUDAKernel class
      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaMemcpyAsync(xHostPointer, gpuXHostPointer, (long) gpuXHostPointer.sizeof() * xHostPointerSize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(yHostPointer, gpuYHostPointer, (long) gpuYHostPointer.sizeof() * yHostPointerSize, cudaMemcpyDefault, stream);

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

      cudart.cudaStreamDestroy(stream);

      if (stream != null)
      {
         stream.close();
      }

      return new float[] {xResult[0], yResult[0]};
   }

   private float[] runCoordinateToIndicesOnOpenCL()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      _cl_program openCLProgram = openCLManager.loadProgram("HeightMapUtilsTest", "HeightMapUtils.cl");
      _cl_kernel openCLKernel = openCLManager.createKernel(openCLProgram, "test_coordinate_to_indices");

      // This creates the (global int* index) parameter used in the kernel.
      // Then this adds the data inside indexPointer to be the first argument for the kernel
      _cl_mem indexPointerObject = openCLManager.createBufferObject(intBytes, indexPointer);
      openCLManager.enqueueWriteBuffer(indexPointerObject, intBytes, indexPointer);
      openCLManager.setKernelArgument(openCLKernel, 0, indexPointerObject);

      // We need to create the 6 parameters used in the kernel.
      // The following sequence is repeated to create all the parameters correctly.
      _cl_mem centerPointerObject = openCLManager.createBufferObject(floatBytes, centerPointer);
      openCLManager.enqueueWriteBuffer(centerPointerObject, floatBytes, centerPointer);
      openCLManager.setKernelArgument(openCLKernel, 1, centerPointerObject);

      _cl_mem resolutionPointerObject = openCLManager.createBufferObject(floatBytes, resolutionPointer);
      openCLManager.enqueueWriteBuffer(resolutionPointerObject, floatBytes, resolutionPointer);
      openCLManager.setKernelArgument(openCLKernel, 2, resolutionPointerObject);

      _cl_mem centerIndexPointerObject = openCLManager.createBufferObject(intBytes, centerIndexPointer);
      openCLManager.enqueueWriteBuffer(centerIndexPointerObject, intBytes, centerIndexPointer);
      openCLManager.setKernelArgument(openCLKernel, 3, centerIndexPointerObject);

      _cl_mem xHostPointerObject = openCLManager.createBufferObject(floatBytes, xHostPointer);
      openCLManager.enqueueWriteBuffer(xHostPointerObject, floatBytes, xHostPointer);
      openCLManager.setKernelArgument(openCLKernel, 4, xHostPointerObject);

      _cl_mem yHostPointerObject = openCLManager.createBufferObject(floatBytes, yHostPointer);
      openCLManager.enqueueWriteBuffer(yHostPointerObject, floatBytes, yHostPointer);
      openCLManager.setKernelArgument(openCLKernel, 5, yHostPointerObject);

      openCLManager.execute1D(openCLKernel, 1);

      openCLManager.enqueueReadBuffer(xHostPointerObject, floatBytes, xHostPointer);
      openCLManager.enqueueReadBuffer(yHostPointerObject, floatBytes, yHostPointer);

      openCLProgram.close();
      openCLManager.destroy();

      return new float[] {xHostPointer.get(0), yHostPointer.get(0)};
   }


   private float[] runCoordinateToIndicesOnCUDA()
   {
      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuHeightMap/HeightMapUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuHeightMap/HeightMapUtils.cuh");

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

      cudaMallocAsync(gpuCoordinatePointer, gpuCoordinatePointer.sizeof() * centerPointerSize, stream);
      cudaMallocAsync(gpuCenterPointer, gpuCenterPointer.sizeof() * resolutionPointerSize, stream);
      cudaMallocAsync(gpuXHostPointer, gpuXHostPointer.sizeof() * xHostPointerSize, stream);
      cudaMallocAsync(gpuYHostPointer, gpuYHostPointer.sizeof() * yHostPointerSize, stream);

      cudaMemcpyAsync(gpuCoordinatePointer, coordinatePointer, (long) centerPointer.sizeof() * centerPointerSize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(gpuCenterPointer, centerPointer, (long) centerPointer.sizeof() * centerPointerSize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(gpuXHostPointer, xHostPointer, (long) xHostPointer.sizeof() * xHostPointerSize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(gpuYHostPointer, yHostPointer, (long) yHostPointer.sizeof() * yHostPointerSize, cudaMemcpyDefault, stream);

      kernel.withPointer(gpuCoordinatePointer).withPointer(gpuCenterPointer);
      kernel.withFloat(5.0f).withInt(5);
      kernel.withPointer(gpuXHostPointer).withPointer(gpuYHostPointer);

      // This is where the kernel is run, for more help with CUDA Examples look at the ExampleCUDAKernel class
      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaMemcpyAsync(xHostPointer, gpuXHostPointer, (long) gpuXHostPointer.sizeof() * xHostPointerSize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(yHostPointer, gpuYHostPointer, (long) gpuYHostPointer.sizeof() * yHostPointerSize, cudaMemcpyDefault, stream);

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

      cudart.cudaStreamDestroy(stream);

      if (stream != null)
      {
         stream.close();
      }

      return new float[] {xResult[0], yResult[0]};
   }
}