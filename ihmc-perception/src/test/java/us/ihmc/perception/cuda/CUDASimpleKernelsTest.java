package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.junit.jupiter.api.Test;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.cudart.cudaMemcpyDefault;
import static org.junit.jupiter.api.Assertions.*;

public class CUDASimpleKernelsTest
{
   @Test
   public void testPassVariableToGPU()
   {
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");

      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(kernelPath);
      CUDAKernel kernel = program.loadKernel("pass_in_variable");

      FloatPointer gpuResult = new FloatPointer();
      cudaMallocAsync(gpuResult, gpuResult.sizeof(), stream);

      kernel.withInt(5).withPointer(gpuResult);
      kernel.run(stream, new dim3(), new dim3(), 0);

      FloatPointer cpuResult = new FloatPointer(1F);
      cudaMemcpyAsync(cpuResult, gpuResult, cpuResult.sizeof(), cudaMemcpyDefault, stream);
      cudaStreamSynchronize(stream);

      int expectedValue = 5; // This is the value passed into the kernel as the first parameter in (kernel.withInt(5))
      assertEquals(expectedValue, cpuResult.get(0), "The expected value is: " + expectedValue
                                                    + " but the actual result is: " + cpuResult.get(0));

      cudaFreeAsync(gpuResult, stream);

      gpuResult.close();
      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }

   /**
    * Here the goal is to pass in the wrong number of variables and see that the GPU crashes accordingly.
    * This test shows what happens when you make that mistake.
    */
   // TODO: this test fails when releasing the stream. That seems wrong because there are 6 parameters being passed into the kernel
   // TODO: So it would seem like the kernel should not even run, but it does... (this could use some investigating)
   @Test
   public void testWrongNumberOfKernelVariables()
   {
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");
      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(kernelPath);

      CUDAKernel kernel = program.loadKernel("pass_in_variable");

      // Pass in way to many variables and see what happens
      kernel.withInt(5).withFloat(4.0f).withLong((long) 2.0);
      kernel.withInt(5).withFloat(4.0f).withLong((long) 2.0);

      // TODO how is this not failing when the kernel is trying to pass in a lot of variables that don't get defined in the kernel on the GPU
      kernel.run(stream, new dim3(), new dim3(), 0);

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }


   /**
    * A stream needs to be created before creating a CUDAProgram. This should throw an exception if the program is created first.
    * This test checks that an exception is thrown and that `cudaErrorDeviceUninitialized` is in the message
    */
   @Test
   public void testCreateStreamAfterManager()
   {
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
   void testNoExternCInGPUKernel()
   {
      URL kernelPath = getClass().getResource("CUDASimpleKernels.cu");
      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(kernelPath);

      // Since we haven't declared the (extern "C"), we expect this kernel to fail loading
      RuntimeException thrown = assertThrows(RuntimeException.class, () -> program.loadKernel("kernel_not_declared_correctly"));
      assertTrue(thrown.getMessage().contains("cudaErrorSymbolNotFound"));

      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
