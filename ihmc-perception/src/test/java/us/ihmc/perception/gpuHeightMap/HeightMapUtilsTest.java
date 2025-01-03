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
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAProgramTest;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.opencl.OpenCLManager;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.cudart.cudaMemcpyAsync;

public class HeightMapUtilsTest
{
   private final int floatBytes = Float.BYTES;
   private final int intBytes = Integer.BYTES;
   private IntPointer indexPointer;
   private FloatPointer centerPointer;
   private FloatPointer resolutionPointer;
   private IntPointer centerIndexPointer;
   private FloatPointer xHostPointer;
   private FloatPointer yHostPointer;

   @BeforeEach
   public void setupPointers()
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

   @AfterEach
   public void closePointers()
   {
      indexPointer.close();
      centerPointer.close();
      resolutionPointer.close();
      centerIndexPointer.close();
      xHostPointer.close();
      yHostPointer.close();
   }

   @Test
   public void testIndexToCoordinateOpenCL()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      _cl_program openCLProgram = openCLManager.loadProgram("HeightMapUtilsTest", "HeightMapUtils.cl");
      _cl_kernel openCLKernel = openCLManager.createKernel(openCLProgram, "test_indices_to_coordinate");

      // This creates the (global int* index) parameter used in the kernel.
      // Then this adds the data inside indexPointer to be the first argument for the kernel
      _cl_mem indexPointerObject = openCLManager.createBufferObject(intBytes, indexPointer);
      openCLManager.enqueueWriteBuffer(indexPointerObject, intBytes, indexPointer);
      openCLManager.setKernelArgument(openCLKernel, 0, indexPointerObject);

      // This creates the (global float* center) parameter used in the kernel.
      // Then this adds the data inside centerPointer to be the first argument for the kernel
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

      float expectedXValueFromKernel = (indexPointer.get(0) - centerIndexPointer.get()) * resolutionPointer.get(0) + centerPointer.get(0);
      int yIndexValue = 1;// Represents the y value in indexForConversion
      float expectedYValueFromKernel = (yIndexValue - centerIndexPointer.get()) * resolutionPointer.get(0) + centerPointer.get(0);

      Assert.assertEquals((int) expectedXValueFromKernel, (int) xHostPointer.get(0));
      Assert.assertEquals((int) expectedYValueFromKernel, (int) yHostPointer.get(0));

      LogTools.info(xHostPointer.get(0) + " is the X result!");
      LogTools.info(expectedXValueFromKernel + " is the X expected value!");
      LogTools.info(yHostPointer.get(0) + " is the Y result!");
      LogTools.info(expectedYValueFromKernel + " is the Y expected value!");

      openCLProgram.close();
      openCLManager.destroy();
   }

   @Test
   public void testIndexToCoordinateCUDA()
   {
      CUstream_st stream = CUDAStreamManager.getStream();

      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuHeightMap/HeightMapUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuHeightMap/HeightMapUtils.cuh");

      CUDAProgram program = new CUDAProgram(programPath, headerPath);

      CUDAKernel kernel = program.loadKernel("test_indices_to_coordinate");

      // These will be pointers to the gpu memory, where we will upload the data too.
      FloatPointer gpuIndexPointer = new FloatPointer();
      FloatPointer gpuCenterPointer = new FloatPointer();
      FloatPointer gpuResolutionPointer = new FloatPointer();
      FloatPointer gpuCenterIndexPointer = new FloatPointer();
      FloatPointer gpuXHostPointer = new FloatPointer();
      FloatPointer gpuYHostPointer = new FloatPointer();

      long indexPointerSize = indexPointer.limit() + 1;
      long centerPointerSize = centerPointer.limit() + 1;
      long resolutionPointerSize = resolutionPointer.limit() + 1;
      long centerIndexPointerSize = centerIndexPointer.limit() + 1;
      long xHostPointerSize = xHostPointer.limit() + 1;
      long yHostPointerSize = yHostPointer.limit() + 1;

      cudaMallocAsync(gpuIndexPointer, gpuIndexPointer.sizeof() * indexPointerSize, stream);
      cudaMallocAsync(gpuCenterPointer, gpuCenterPointer.sizeof() * centerPointerSize, stream);
      cudaMallocAsync(gpuResolutionPointer, gpuResolutionPointer.sizeof() * resolutionPointerSize, stream);
      cudaMallocAsync(gpuCenterIndexPointer, gpuCenterIndexPointer.sizeof() * centerIndexPointerSize, stream);
      cudaMallocAsync(gpuXHostPointer, gpuXHostPointer.sizeof() * xHostPointerSize, stream);
      cudaMallocAsync(gpuYHostPointer, gpuYHostPointer.sizeof() * yHostPointerSize, stream);

      int cudaDefaultValue = cudaMemcpyDefault;

      cudaMemcpyAsync(gpuIndexPointer, indexPointer, (long) indexPointer.sizeof() * indexPointerSize, cudaDefaultValue, stream);
      cudaMemcpyAsync(gpuCenterPointer, centerPointer, (long) centerPointer.sizeof() * centerPointerSize, cudaDefaultValue, stream);
      cudaMemcpyAsync(gpuResolutionPointer, resolutionPointer, (long) resolutionPointer.sizeof() * resolutionPointerSize, cudaDefaultValue, stream);
      cudaMemcpyAsync(gpuCenterIndexPointer, centerIndexPointer, (long) centerIndexPointer.sizeof() * centerIndexPointerSize, cudaDefaultValue, stream);
      cudaMemcpyAsync(gpuXHostPointer, xHostPointer, (long) xHostPointer.sizeof() * xHostPointerSize, cudaDefaultValue, stream);
      cudaMemcpyAsync(gpuYHostPointer, yHostPointer, (long) yHostPointer.sizeof() * yHostPointerSize, cudaDefaultValue, stream);

      kernel.withInt(5).withPointer(gpuCenterPointer).withPointer(gpuResolutionPointer);
      kernel.withInt(5).withPointer(gpuXHostPointer).withPointer(gpuYHostPointer);

      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaMemcpyAsync(xHostPointer, gpuXHostPointer, (long) gpuXHostPointer.sizeof() * xHostPointerSize, cudaDefaultValue, stream);
      cudaMemcpyAsync(yHostPointer, gpuYHostPointer, (long) gpuYHostPointer.sizeof() * yHostPointerSize, cudaDefaultValue, stream);

      cudaStreamSynchronize(stream);

      float[] xResult = new float[1];
      float[] yResult = new float[1];
      xHostPointer.get(xResult);
      yHostPointer.get(yResult);

      LogTools.info("xResult: " + xResult[0]);
      LogTools.info("yResult: " + yResult[0]);

      // Free the memory on the GPU now that we are done with it. The data in on the CPU so we don't need it anymore
      cudaFreeAsync(gpuCenterPointer, stream);
      cudaFreeAsync(gpuCenterPointer, stream);
      cudaFreeAsync(gpuResolutionPointer, stream);
      cudaFreeAsync(gpuCenterIndexPointer, stream);
      cudaFreeAsync(gpuXHostPointer, stream);
      cudaFreeAsync(gpuYHostPointer, stream);

      kernel.close();
      program.close();

      gpuIndexPointer.close();
      gpuCenterPointer.close();
      gpuResolutionPointer.close();
      gpuCenterIndexPointer.close();
      gpuXHostPointer.close();
      gpuYHostPointer.close();

      cudart.cudaStreamDestroy(stream);
      stream.close();
   }
}
