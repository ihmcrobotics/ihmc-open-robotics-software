package us.ihmc.perception.cuda;

import com.google.monitoring.runtime.instrumentation.common.primitives.Bytes;
import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.net.URISyntaxException;
import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;
import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.perception.tools.PerceptionDebugTools.printMat;

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

   private static final String ADD_VECTOR_KERNEL = """        
         extern "C"
         
         __global__ void add_vector(float *a, int *b, int*sum) 
         {
             int idx = blockIdx.x * blockDim.x + threadIdx.x;
             if (idx < 7) 
             {
             sum[idx] = a[idx] + b[idx];
             }
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
   private static final String ADD_MATRICES_KERNEL = """
         extern "C"
         __global__ void add_matrices(int* matA, int* matB, int* result) 
         {
             int x = blockIdx.x * blockDim.x + threadIdx.x;
             int y = blockIdx.y * blockDim.y + threadIdx.y;
         
             // Calculate the pixel index in the flattened array
             int idx = (y * 2 + x) * 1;
         
             if (x < 2 && y < 2) {
                 for (int c = 0; c < 1; ++c) {
                     result[idx + c] = matA[idx + c] + matB[idx + c];
                 }
             }
         }
         """;

   private void checkCudaError(String errorMsg)
   {
      int errorCode = cudaGetLastError();
      if (errorCode != cudaSuccess)
      {
         System.err.println(errorMsg + ": " + cudaGetErrorString(errorCode));
         throw new RuntimeException(errorMsg);
      }
   }

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
         // FIXME should this be run after the kernel and before the memcpy rather than after the memcpy? If it is synchronizing the GPU kernels, then it should
         // be run after the kernel. Otherwise, you don't know that all the kernel threads are finished when you start copying the data from the GPU to the CPU
         // with memcpy.
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
   public void testSimpleVectorKernel()
   {
      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram additionProgram = new CUDAProgram("add_vector.cu", ADD_VECTOR_KERNEL);

      // Construct a program

      // Load the kernel
      additionProgram.loadKernel("add_vector");

      long vectorSize = 7;

      float[] hostArrayA = {10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f};
      FloatPointer hostPointerA = new FloatPointer(vectorSize);
      hostPointerA.put(hostArrayA);

      int[] hostArrayB = {70, 60, 50, 40, 30, 20, 10};
      IntPointer hostPointerB = new IntPointer(vectorSize);
      hostPointerB.put(hostArrayB);

      Pointer deviceA = new Pointer();
      cudaMallocAsync(deviceA, vectorSize * Integer.BYTES, stream);
      cudaMemcpyAsync(deviceA, hostPointerA, vectorSize * Integer.BYTES, cudaMemcpyHostToDevice, stream);

      Pointer deviceB = new Pointer();
      cudaMallocAsync(deviceB, vectorSize * Integer.BYTES, stream);
      cudaMemcpyAsync(deviceB, hostPointerB, vectorSize * Integer.BYTES, cudaMemcpyHostToDevice, stream);

      Pointer deviceSum = new Pointer();
      cudaMallocAsync(deviceSum, vectorSize * Integer.BYTES, stream);

      PointerPointer<Pointer> deviceAPointer = new PointerPointer<>(1L);
      PointerPointer<Pointer> deviceBPointer = new PointerPointer<>(1L);
      PointerPointer<Pointer> deviceSumPointer = new PointerPointer<>(1L);

      deviceAPointer.put(deviceA);
      deviceBPointer.put(deviceB);
      deviceSumPointer.put(deviceSum);

      additionProgram.runKernel(stream, "add_vector", new dim3(1, 1, 1), new dim3(7, 1, 1), 0, deviceAPointer, deviceBPointer, deviceSumPointer);
      IntPointer sum = new IntPointer(vectorSize);
      cudaStreamSynchronize(stream);

      cudaMemcpyAsync(sum, deviceSum, vectorSize * Integer.BYTES, cudaMemcpyDeviceToHost, stream);

      for (int i = 0; i < 7; i++)
         assertEquals(80, sum.get(i));

      hostPointerA.deallocate();
      hostPointerB.deallocate();
      sum.deallocate();

      cudaFreeAsync(deviceA, stream);
      cudaFreeAsync(deviceB, stream);
      cudaFreeAsync(deviceSum, stream);

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
