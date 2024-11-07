package us.ihmc.perception.cuda;

import com.google.monitoring.runtime.instrumentation.common.primitives.Bytes;
import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;

import org.bytedeco.opencv.global.opencv_core;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.net.URISyntaxException;
import java.nio.file.Path;
import java.util.Objects;

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
         // FIXME should this be run after the kernel and before the memcpy rather than after the memcpy? If it is synchronizing the GPU kernels, then it should
         // be run after the kernel. Otherwise, you don't know that all the kernel threads are finished when you start copying the data from the GPU to the CPU
         // with memcpy.
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
   public void testMatrixSum()
   {
      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram additionProgram = new CUDAProgram("add_matrices.cu", ADD_MATRICES_KERNEL);
      additionProgram.loadKernel("add_matrices");

      // Example: 640x480 image, single-channel or multi-channel
      int width = 2;
      int height = 2;
      int channels = 1; // e.g., RGB image

      // Create and fill OpenCV Mats for the two input matrices
      Mat matA = new Mat(height,width,opencv_core.CV_16UC1); // RGB format
      Mat matB = new Mat(height, width,opencv_core.CV_16UC1);
      // Randomly fill the matrices (or load images)
      System.out.println(matA.getPointer());

      matA.ptr(0, 0).putInt(1);
      matA.ptr(0, 1).putInt(2);
      matA.ptr(1, 0).putInt(3);
      matA.ptr(1, 1).putInt(4);
//
//      matB.ptr(0, 0).putInt(1);
//      matB.ptr(0, 1).putInt(2);
//      matB.ptr(1, 0).putInt(3);
//      matB.ptr(1, 1).putInt(4);
//
//      System.out.println("here");
//      System.out.println(matA.getPointer(0));
//      System.out.println("here");



//       Allocate GPU memory and copy data from host (CPU) to device (GPU)
//      long numBytes = width * height * channels * Integer.BYTES;
//
//      //make host pointers
//      BytePointer hostPointerA = new BytePointer(numBytes);
//      BytePointer hostPointerB = new BytePointer(numBytes);
//      BytePointer hostPointerSum = new BytePointer(numBytes);
//
//      //fill host pointers
//      hostPointerA.put(matA);
//      hostPointerB.put(matB);
//
//      //make device pointers
//      Pointer deviceA = new Pointer();
//      Pointer deviceB = new Pointer();
//      Pointer deviceSum = new Pointer();
////
//      //assign memory for device pointers
//      cudaMallocAsync(deviceA, numBytes, stream);
//      cudaMallocAsync(deviceB, numBytes, stream);
//      cudaMallocAsync(deviceSum, numBytes, stream);
////
//      //copy input to memory of gpu
//      cudaMemcpyAsync(deviceA, hostPointerA, numBytes, cudaMemcpyHostToDevice, stream);
//      cudaMemcpyAsync(deviceB, hostPointerB, numBytes, cudaMemcpyHostToDevice, stream);
//
//      //make device pointers
//      PointerPointer<Pointer> deviceAPointer = new PointerPointer<>(1L);
//      PointerPointer<Pointer> deviceBPointer = new PointerPointer<>(1L);
//      PointerPointer<Pointer> deviceSumPointer = new PointerPointer<>(1L);
//
//      //put device pointers in pointer pointers
//      deviceAPointer.put(deviceA);
//      deviceBPointer.put(deviceB);
//      deviceSumPointer.put(deviceSum);
//
//      //run the kernel
//      additionProgram.runKernel(stream, "add_matrices", new dim3(1, 1, 1), new dim3(2, 2, 1), 0, deviceAPointer, deviceBPointer, deviceSumPointer);
//      cudaStreamSynchronize(stream);
//      //copy from device pointer to host pointer
//      cudaMemcpyAsync(hostPointerSum, deviceSum, numBytes, cudaMemcpyDeviceToHost, stream);
//
//      System.out.println(hostPointerSum.get(0));
//      System.out.println(hostPointerSum.get(1));
//      System.out.println(hostPointerSum.get(2));
//      System.out.println(hostPointerSum.get(3));
   }

//
//      for (int i = 0; i < 7; i++)
//         assertEquals(80, sum.get(i));
//
//      hostPointerA.deallocate();
//      hostPointerB.deallocate();
//      sum.deallocate();
//
//      cudaFreeAsync(deviceA, stream);
//      cudaFreeAsync(deviceB, stream);
//      cudaFreeAsync(deviceSum, stream);
//
//      additionProgram.destroy();
//
//      CUDAStreamManager.releaseStream(stream);
//   }
//
//      // Configure kernel launch parameters
//
//      // Set up and launch kernel
//      cudaStream_t stream = new cudaStream_t();
//      cudaStreamCreate(stream);
//      CUmodule module = new CUmodule();
//      CUfunction function = new CUfunction();
//
//      additionProgram.loadKernel("add_matrices");
//      additionProgram.runKernel(stream, "add_matrices", grid, block, 0, deviceA, deviceB, deviceResult, width, height, channels);
//
//      cudaStreamSynchronize(stream);
//
//      // Copy result back to host
//      Mat resultMat = new Mat(height, width, opencv_core.CV_8UC3);
//      cudaMemcpy(resultMat.data(), deviceResult, numBytes, cudaMemcpyDeviceToHost);
//
//      // Clean up
//      cudaFree(deviceA);
//      cudaFree(deviceB);
//      cudaFree(deviceResult);
//      cudaStreamDestroy(stream);
//
//      // Display or further process resultMat if desired
//      System.out.println("Matrix addition completed on GPU!");
//
//      // Optional: Display result (if you have GUI capabilities)
//      // imshow("Result", resultMat);
//      // waitKey();
//   }
//}
//   }

//@Disabled
//@Test
//public void testVectorSum()
//{
//   // TODO we want to write a vector element sum kernel, where each of the elements of the vector get added together and returned from the GPU. This requires
//   // memory blocking to execute properly, forcing each kernel to run sequentially, rather than in parallel. It is a stupid operation to do on the GPU, but
//   // provides an informative example for how to do this kind of thing.
//}

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
   try (IntPointer sum = new IntPointer(1L); IntPointer deviceSum = new IntPointer(); PointerPointer<Pointer> deviceSumPointer = new PointerPointer<>(1L))
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
