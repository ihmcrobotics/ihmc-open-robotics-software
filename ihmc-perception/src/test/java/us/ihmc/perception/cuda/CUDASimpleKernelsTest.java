package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

import static org.junit.jupiter.api.Assertions.*;

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
   public void testPassVariableToGPU() throws Exception
   {
      URL kernelPath = getClass().getResource("/cuda/CUDASimpleKernels.cu");

      stream = CUDAStreamManager.getStream();
      program = new CUDAProgram(kernelPath);
      kernel = program.loadKernel("pass_in_variable");

      FloatPointer gpuResult = new FloatPointer();

      // The convention is to use CUDATools for allocating memory at the moment
      CUDATools.mallocAsync(gpuResult, gpuResult.sizeof(), stream);
      // cudaMallocAsync(gpuResult, gpuResult.sizeof(), stream);

      kernel.withInt(5).withPointer(gpuResult);
      kernel.run(stream, new dim3(), new dim3(), 0);

      FloatPointer cpuResult = new FloatPointer(1F);

      // The convention is to use CUDATools for allocating memory at the moment
      CUDATools.memcpyAsync(cpuResult, gpuResult, cpuResult.sizeof(), stream);
      // cudaMemcpyAsync(cpuResult, gpuResult, cpuResult.sizeof(), cudaMemcpyDefault, stream);
      cudaStreamSynchronize(stream);

      int expectedValue = 5; // This is the value passed into the kernel as the first parameter in (kernel.withInt(5))
      assertEquals(expectedValue, cpuResult.get(0), "The expected value is: " + expectedValue + " but the actual result is: " + cpuResult.get(0));

      cudaFreeAsync(gpuResult, stream);

      gpuResult.close();
   }

   /**
    * Test that when you pass in the wrong type into the GPU kernel that it fails.
    * WHen the kernel expects and int, and you pass an in float, it should fail.
    * In this case, we pass in a float, but it expects an int.
    * So the gpu converts the int to a float in bits.
    * Then it uses that value for the duration of the kernel.
    * This value is returned and compared against the expected result
    */
   @Test
   public void testPassingInWrongTypeToGPU() throws Exception
   {
      URL kernelPath = getClass().getResource("/cuda/CUDASimpleKernels.cu");
      stream = CUDAStreamManager.getStream();
      program = new CUDAProgram(kernelPath);
      kernel = program.loadKernel("pass_in_variable");

      FloatPointer gpuResult = new FloatPointer();
      CUDATools.mallocAsync(gpuResult, gpuResult.sizeof(), stream);

      float originalValue = 5.0f; // This is the value passed into the kernel as the first parameter in (kernel.withInt(5))
      kernel.withFloat(originalValue).withPointer(gpuResult);

      kernel.run(stream, new dim3(), new dim3(), 0);

      FloatPointer cpuResult = new FloatPointer(1F);
      CUDATools.memcpyAsync(cpuResult, gpuResult, cpuResult.sizeof(), stream);
      cudaStreamSynchronize(stream);

      // Since we are passing in a float into the kernel, and it expected an int.
      // It converted the float to an int as bits.
      float floatBits = Float.floatToIntBits(originalValue);
      String expectedStringValue = Float.toString(floatBits);

      String cpuResultAsString = Float.toString(cpuResult.get(0));
      assertEquals(expectedStringValue, cpuResultAsString, "The expected value is: " + expectedStringValue + " but the actual result is: " + cpuResultAsString);

      cudaFreeAsync(gpuResult, stream);
      gpuResult.close();
   }

   /**
    * Here the goal is to pass in the wrong number of variables and see that the GPU crashes accordingly.
    * This test shows what happens when you make that mistake.
    * CUDA reports these errors in later CUDA function calls (once it knows there was an error).
    * So that's why we see the errors from the {@link CUDAStreamManager}.
    * This test is disabled because the stream doesn't get cleared correctly, and it affects the next time a stream is created
    */
   @Disabled
   @Test
   public void testWrongNumberOfKernelVariables() throws Exception
   {
      URL kernelPath = getClass().getResource("/cuda/CUDASimpleKernels.cu");

      // This test creates its own stream to not interfere with other tests
      CUstream_st streamLocal = new CUstream_st();
      program = new CUDAProgram(kernelPath);
      kernel = program.loadKernel("pass_in_variable");

      // Pass in way to many variables and see what happens
      kernel.withInt(5).withFloat(4.0f).withLong((long) 2.0);
      kernel.withInt(5).withFloat(4.0f).withLong((long) 2.0);

      // Even though the kernel has way to many parameters, we can still run it.
      // This is because we call it asynchronously and so it may not run right away,
      // However, when we synchronize the stream we block until the kernel has run, so that will report the error
      kernel.run(streamLocal, new dim3(), new dim3(), 0);

      // This forces the gpu code to run as the stream is waiting for the kernels to finish.
      // This is when the error is reported
      // Note that the error is NOT reported till you synchronize the stream, that isn't always obvious
      RuntimeException thrown = assertThrows(RuntimeException.class, () -> CUDATools.checkCUDAError(cudaStreamSynchronize(streamLocal)));

      assertTrue(thrown.getMessage().contains("an illegal memory access was encountered"));

      // TODO there is a problem with releasing the stream, so even if the following lines are performed
      // Any call to cudart.cudaStreamCreate(stream) will throw an error
      streamLocal.position(0);
      streamLocal.deallocate();
      streamLocal.close();
   }

   /**
    * When writing a CUDA kernel, each kernel needs to declare (extern "C") just above the kernel.
    * If this doesn't happen, that kernel should fail as it won't be loaded correctly.
    * Making sure that if fails here...
    */
   @Test
   public void testNoExternCInGPUKernel() throws Exception
   {
      URL kernelPath = getClass().getResource("/cuda/CUDASimpleKernels.cu");

      stream = CUDAStreamManager.getStream();
      program = new CUDAProgram(kernelPath);

      // Since we haven't declared the (extern "C"), we expect this kernel to fail loading
      Exception thrown = assertThrows(Exception.class, () -> program.loadKernel("kernel_not_declared_correctly"));
      assertTrue(thrown.getMessage().contains("cudaErrorSymbolNotFound"));
   }

   @Test
   public void testNoSemicolonInKernel()
   {
      URL kernelPath = getClass().getResource("/cuda/CUDAKernelCodeWrong.cu");

      stream = CUDAStreamManager.getStream();

      // Since we haven't declared the (extern "C"), we expect this kernel to fail loading
      Exception thrown = assertThrows(Exception.class, () ->
      {
         CUDAProgram program = new CUDAProgram(kernelPath);
         program.close();
      });

      assertTrue(thrown.getMessage().contains("cudaErrorInvalidKernelImage"));
   }
}
