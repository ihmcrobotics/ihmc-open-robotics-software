package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.cudart.cudaMemcpyDefault;
import static org.junit.jupiter.api.Assertions.*;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class CUDASimpleKernelsTest
{
   private CUstream_st stream;
   private CUDAProgram program;
   private CUDAKernel kernel;

   @AfterEach
   public void tearDown()
   {
      // We have null checks here because not all tests will create all of these objeccts
      if (kernel != null)
      {
         kernel.close();
      }

      if (program != null)
      {
         program.close();
      }

      if (stream != null)
      {
         CUDAStreamManager.releaseStream(stream);
      }
   }

   @Test
   public void testPassVariableToGPU()
   {
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");

      stream = CUDAStreamManager.getStream();
      program = new CUDAProgram(kernelPath);
      kernel = program.loadKernel("pass_in_variable");

      FloatPointer gpuResult = new FloatPointer();

      // The convention is to use CUDATools for allocating memory at the moment
      CUDATools.mallocAsync(gpuResult, gpuResult.sizeof(), stream);
//      cudaMallocAsync(gpuResult, gpuResult.sizeof(), stream);

      kernel.withInt(5).withPointer(gpuResult);
      kernel.run(stream, new dim3(), new dim3(), 0);

      FloatPointer cpuResult = new FloatPointer(1F);

      // The convention is to use CUDATools for allocating memory at the moment
      CUDATools.memcpyAsync(cpuResult, gpuResult, cpuResult.sizeof(), stream);
//      cudaMemcpyAsync(cpuResult, gpuResult, cpuResult.sizeof(), cudaMemcpyDefault, stream);
      cudaStreamSynchronize(stream);

      int expectedValue = 5; // This is the value passed into the kernel as the first parameter in (kernel.withInt(5))
      assertEquals(expectedValue, cpuResult.get(0), "The expected value is: " + expectedValue + " but the actual result is: " + cpuResult.get(0));

      cudaFreeAsync(gpuResult, stream);

      gpuResult.close();
   }

   /**
    * Test that when you pass in the wrong type into the GPU kernel that it fails.
    * WHen the kernel expects and int and you pass a in float, it should fail.
    */
   // TODO this test should fail but seems to pass even when the wrong paramter is passed into the kernel
   @Disabled
   @Test
   public void testPassingInWrongTypeToGPU()
   {
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");
      stream = CUDAStreamManager.getStream();
      program = new CUDAProgram(kernelPath);
      kernel = program.loadKernel("pass_in_int");

      kernel.withFloat(5.0f);

      // TODO how do we handle the case where the kernel can run but the parameters are wrong?
      // TODO is that something we can change or is that just the way it is with gpu programming?... (someone investigate this)
      kernel.run(stream, new dim3(), new dim3(), 0);

      //      RuntimeException thrown = assertThrows(RuntimeException.class, () -> kernel.run(stream, new dim3(), new dim3(), 0));
      //      assertTrue(thrown.getMessage().contains("cudaErrorInvalidValue"));

      cudaStreamSynchronize(stream);
   }

   /**
    * Here the goal is to pass in the wrong number of variables and see that the GPU crashes accordingly.
    * This test shows what happens when you make that mistake.
    */
   // TODO: this test fails when releasing the stream. That seems wrong because there are 6 parameters being passed into the kernel
   // TODO: So it would seem like the kernel should not even run, but it does... (this could use some investigating)
   // TODO: This also causes other tests to fail when this one doesn't release the steam correclty... bugs in the code lol
   @Disabled
   @Test
   public void testWrongNumberOfKernelVariables()
   {
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");
      stream = CUDAStreamManager.getStream();
      program = new CUDAProgram(kernelPath);
      kernel = program.loadKernel("pass_in_variable");

      // Pass in way to many variables and see what happens
      kernel.withInt(5).withFloat(4.0f).withLong((long) 2.0);
      kernel.withInt(5).withFloat(4.0f).withLong((long) 2.0);

      // TODO how is this not failing when the kernel is trying to pass in a lot of variables that don't get defined in the kernel on the GPU
      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaStreamSynchronize(stream);
      // ^^^ TODO How come this doesn't fix things???
   }

   /**
    * This test is meant to test the case where you don't synchronize the stream after running the kernel.
    * Need to do some more testing to see why this still works, and if there is a better way to test this.
    * That is why this test is disabled at the moment, it doesn't appear to be that useful
    */
   @Disabled
   @Test
   public void testNotSynchronizingStream()
   {
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");
      stream = CUDAStreamManager.getStream();
      program = new CUDAProgram(kernelPath);
      kernel = program.loadKernel("pass_in_int");

      kernel.withInt(5);

      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaStreamSynchronize(stream);
   }

   /**
    * A stream needs to be created before creating a CUDAProgram. This should throw an exception if the program is created first.
    * This test checks that an exception is thrown and that `cudaErrorDeviceUninitialized` is in the message
    */
   // Needed to have this test run first because if this test doesn't run first it will fail.
   // TODO this is not good as independent order is important as these tests should be independent
   @Order(1)
   @Test
   public void testCreateStreamAfterManager()
   {
      // In order for this test to pass the stream needs to be null
      stream = null;
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");
      RuntimeException thrown = assertThrows(RuntimeException.class, () ->
      {
         CUDAProgram program = new CUDAProgram(kernelPath);
         program.close();
      });

      assertTrue(thrown.getMessage().contains("cudaErrorDeviceUninitialized"));
   }

   /**
    * When writing a CUDA kernel, each kernel needs to declare (extern "C") just above the kernel.
    * If this doesn't happen, that kernel should fail as it won't be loaded correctly.
    * Making sure that if fails here...
    */
   @Test
   public void testNoExternCInGPUKernel()
   {
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");

      stream = CUDAStreamManager.getStream();
      program = new CUDAProgram(kernelPath);

      // Since we haven't declared the (extern "C"), we expect this kernel to fail loading
      RuntimeException thrown = assertThrows(RuntimeException.class, () -> program.loadKernel("kernel_not_declared_correctly"));
      assertTrue(thrown.getMessage().contains("cudaErrorSymbolNotFound"));
   }

   @Test
   public void testNoSemicolonInKernel()
   {
      URL kernelPath = getClass().getResource("CUDAKernelCodeWrong.cu");

      stream = CUDAStreamManager.getStream();

      // Since we haven't declared the (extern "C"), we expect this kernel to fail loading
      RuntimeException thrown = assertThrows(RuntimeException.class, () ->
      {
         CUDAProgram program = new CUDAProgram(kernelPath);
         program.close();
      });

      assertTrue(thrown.getMessage().contains("NVRTC_ERROR_COMPILATION"));
   }
}
