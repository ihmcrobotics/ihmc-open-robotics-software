package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.IntPointer;
import org.junit.jupiter.api.Test;

import java.net.URISyntaxException;
import java.net.URL;

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

      try (CUDAProgram additionProgram = new CUDAProgram("add.cu", ADD_KERNEL); // Construct the program
           CUDAKernel additionKernel = additionProgram.loadKernel("add");         // Load the kernel
           IntPointer sum = new IntPointer(1L);
           IntPointer deviceSum = new IntPointer())
      {
         cudaMallocAsync(deviceSum, deviceSum.sizeof(), stream);

         additionKernel.withInt(3).withInt(7).withPointer(deviceSum);
         additionKernel.run(stream, new dim3(), new dim3(), 0);

         // Copy result from device to host
         cudaMemcpyAsync(sum, deviceSum, sum.sizeof(), cudaMemcpyDefault, stream);
         cudaStreamSynchronize(stream);

         // Free host memory
         cudaFreeAsync(deviceSum, stream);

         // Ensure we got the correct result!
         assertEquals(10, sum.get());
      }

      CUDAStreamManager.releaseStream(stream);
   }

   @Test
   public void testMultipleKernelRuns()
   {
      int runs = 10;

      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();

      try (CUDAProgram additionProgram = new CUDAProgram("add.cu", ADD_KERNEL); // Construct the program
           CUDAKernel additionKernel = additionProgram.loadKernel("add");         // Load the kernel
           IntPointer sum = new IntPointer(1L);
           IntPointer deviceSum = new IntPointer())
      {
         cudaMallocAsync(deviceSum, deviceSum.sizeof(), stream);

         for (int a = 0; a < runs; ++a)
         {
            for (int b = 0; b < runs; ++b)
            {
               additionKernel.clearParameters();
               additionKernel.withInt(a).withInt(b).withPointer(deviceSum);
               additionKernel.run(stream, new dim3(), new dim3(), 0);

               // Copy result from device to host
               cudaMemcpyAsync(sum, deviceSum, sum.sizeof(), cudaMemcpyDefault, stream);
               cudaStreamSynchronize(stream);

               // Ensure we got the correct result!
               assertEquals(a + b, sum.get());
            }
         }

         // Free host memory
         cudaFreeAsync(deviceSum, stream);
      }

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
      try (CUDAProgram additionProgram = new CUDAProgram("add_header.cu", KERNEL_WITH_HEADER, headerName, headerContents);
           CUDAKernel additionKernel = additionProgram.loadKernel("add"); // Load the kernel
           IntPointer sum = new IntPointer(1L);
           IntPointer deviceSum = new IntPointer())
      {
         cudaMallocAsync(deviceSum, sum.sizeof(), stream);

         // Run the kernel
         additionKernel.withPointer(deviceSum);
         additionKernel.run(stream, new dim3(), new dim3(), 0);

         // Download result from device to host
         cudaMemcpyAsync(sum, deviceSum, sum.sizeof(), cudaMemcpyDefault, stream);
         cudaStreamSynchronize(stream);

         // Free device memory
         cudaFreeAsync(deviceSum, stream);
      }

      CUDAStreamManager.releaseStream(stream);
   }

   @Test
   public void testLoadingKernelFromFile() throws URISyntaxException
   {
      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();

      // Create a CUDA program with files
      URL kernelPath = getClass().getResource("test_add_values.cu");
      URL headerPath = getClass().getResource("test_values.cuh");

      try (CUDAProgram program = new CUDAProgram(kernelPath, headerPath);

           // Load the kernels
           CUDAKernel additionKernel = program.loadKernel("add");
           CUDAKernel subtractionKernel = program.loadKernel("subtract");

           // Create pointers
           IntPointer sum = new IntPointer(1L);
           IntPointer deviceSum = new IntPointer();

           IntPointer difference = new IntPointer(1L);
           IntPointer deviceDifference = new IntPointer())
      {
         cudaMallocAsync(deviceSum, sum.sizeof(), stream);
         cudaMallocAsync(deviceDifference, difference.sizeof(), stream);

         additionKernel.withPointer(deviceSum).run(stream, new dim3(), new dim3(), 0);
         subtractionKernel.withPointer(deviceDifference).run(stream, new dim3(), new dim3(), 0);

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

      CUDAStreamManager.releaseStream(stream);
   }
}