package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.junit.jupiter.api.Test;

import java.net.URISyntaxException;
import java.nio.file.Path;
import java.util.Objects;

import static org.bytedeco.cuda.global.cudart.*;
import static org.junit.jupiter.api.Assertions.*;

public class CUDAProgramTest
{
   private static final String ADD_KERNEL = """
         extern "C"
         __global__
         void add(int a, int b, int * sum)
         {
            *sum = a + b;
         }
         """;

   private static final String KERNEL_HEADER = """
         __constant__ int a = 3;
         __constant__ int b = 7;
         """;

   private static final String KERNEL_WITH_HEADER = """
         #include "test_values.cuh"

         extern "C"
         __global__
         void add(int * sum)
         {
            *sum = a + b;
         }
         """;

   @Test
   public void testSimpleKernel()
   {
      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();

      // Construct a program
      CUDAProgram additionProgram = new CUDAProgram("add.cu", ADD_KERNEL);

      // Load the kernel
      additionProgram.loadKernel("add");

      // Create host & device pointers
      try (IntPointer a = new IntPointer(1L).put(3);
           IntPointer b = new IntPointer(1L).put(7);
           IntPointer sum = new IntPointer(1L);
           IntPointer deviceSum = new IntPointer();
           PointerPointer<Pointer> deviceSumPointer = new PointerPointer<>(1L))
      {
         cudaMallocAsync(deviceSum, sum.sizeof(), stream);
         deviceSumPointer.put(deviceSum);

         // Run the kernel
         additionProgram.runKernel(stream, "add", new dim3(), new dim3(), 0, a, b, deviceSumPointer);

         // Copy result from device to host
         cudaMemcpyAsync(sum, deviceSum, sum.sizeof(), cudaMemcpyDefault, stream);
         cudaStreamSynchronize(stream);

         // Free host memory
         cudaFreeAsync(deviceSum, stream);

         // Ensure we got the correct result!
         assertEquals(10, sum.get());
      }

      additionProgram.destroy();

      CUDAStreamManager.releaseStream(stream);
   }

   @Test
   public void testKernelWithHeader()
   {
      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();

      // Construct a program
      String[] headerName = {"test_values.cuh"};
      String[] headerContents = {KERNEL_HEADER};

      CUDAProgram additionProgram = new CUDAProgram("add_header.cu", KERNEL_WITH_HEADER, headerName, headerContents);

      // Load the kernel
      additionProgram.loadKernel("add");

      // Create pointers
      try (IntPointer sum = new IntPointer(1L);
           IntPointer deviceSum = new IntPointer();
           PointerPointer<Pointer> deviceSumPointer = new PointerPointer<>(1L))
      {
         cudaMallocAsync(deviceSum, sum.sizeof(), stream);
         deviceSumPointer.put(deviceSum);

         // Run the kernel
         additionProgram.runKernel(stream, "add", new dim3(1, 1, 1), new dim3(1, 1, 1), 0, deviceSumPointer);

         // Download result from device to host
         cudaMemcpyAsync(sum, deviceSum, sum.sizeof(), cudaMemcpyDefault, stream);
         cudaStreamSynchronize(stream);

         // Free device memory
         cudaFreeAsync(deviceSum, stream);

         // Ensure we got the correct result!
         assertEquals(10, sum.get());
      }

      additionProgram.destroy();

      CUDAStreamManager.releaseStream(stream);
   }

   @Test
   public void testLoadingKernelFromFile() throws URISyntaxException
   {
      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();

      // Create a CUDA program with files
      Path kernelPath = Path.of(Objects.requireNonNull(getClass().getResource("test_add_values.cu")).toURI());
      Path headerPath = Path.of(Objects.requireNonNull(getClass().getResource("test_values.cuh")).toURI());
      CUDAProgram program = new CUDAProgram(kernelPath, headerPath);

      // Load the kernels
      program.loadKernel("add");
      program.loadKernel("subtract");

      // Create pointers
      try (IntPointer sum = new IntPointer(1L);
           IntPointer deviceSum = new IntPointer();
           PointerPointer<Pointer> deviceSumPointer = new PointerPointer<>(1L);

           IntPointer difference = new IntPointer(1L);
           IntPointer deviceDifference = new IntPointer();
           PointerPointer<Pointer> deviceDifferencePointer = new PointerPointer<>(1L))
      {
         cudaMallocAsync(deviceSum, sum.sizeof(), stream);
         deviceSumPointer.put(deviceSum);
         cudaMallocAsync(deviceDifference, difference.sizeof(), stream);
         deviceDifferencePointer.put(deviceDifference);

         // Run the kernels
         program.runKernel(stream, "add", new dim3(), new dim3(), 0, deviceSumPointer);
         program.runKernel(stream, "subtract", new dim3(), new dim3(), 0, deviceDifferencePointer);

         // Download results from device to host
         cudaMemcpyAsync(sum, deviceSum, sum.sizeof(), cudaMemcpyDefault, stream);
         cudaMemcpyAsync(difference, deviceDifference, difference.sizeof(), cudaMemcpyDefault, stream);
         cudaStreamSynchronize(stream);

         // Free device memory
         cudaFreeAsync(deviceSum, stream);
         cudaFreeAsync(deviceDifference, stream);

         // Ensure we got the correct result!
         assertEquals(10, sum.get());
         assertEquals(4, difference.get());
      }

      program.destroy();

      CUDAStreamManager.releaseStream(stream);
   }
}
