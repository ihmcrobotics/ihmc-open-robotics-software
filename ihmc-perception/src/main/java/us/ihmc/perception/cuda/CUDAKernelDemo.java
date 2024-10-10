package us.ihmc.perception.cuda;

import org.apache.logging.log4j.Level;
import org.bytedeco.cuda.cudart.CUfunc_st;
import org.bytedeco.cuda.cudart.CUkern_st;
import org.bytedeco.cuda.cudart.CUmod_st;
import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.nvrtc._nvrtcProgram;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;

import java.util.stream.Stream;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.nvrtc.*;

public class CUDAKernelDemo
{
   private static String KERNEL = """
                                  extern "C"
                                  __global__ void test(int * data)
                                  {
                                    int threadIndex = blockDim.x * threadIdx.x + threadIdx.y;
                                    data[threadIndex] = threadIndex;
                                  }
                                  """;

   private static BytePointer compileKernelToPTX(String kernelCode,
                                                 String kernelName,
                                                 String[] headerNames,
                                                 String[] headerContents,
                                                 String[] compilationOptions)
   {
      int error;
      try (BytePointer code = new BytePointer(kernelCode);
           BytePointer codeName = new BytePointer(kernelName);
           PointerPointer<BytePointer> headerNamesPointer = headerNames == null ? null : new PointerPointer<>(headerNames);
           PointerPointer<BytePointer> headerContentsPointer = headerContents == null ? null : new PointerPointer<>(headerContents);
           PointerPointer<BytePointer> options = compilationOptions == null ? null : new PointerPointer<>(compilationOptions);

           _nvrtcProgram program = new _nvrtcProgram();
           SizeTPointer ptxSize = new SizeTPointer(1L))
      {
         // Create the program
         error = nvrtcCreateProgram(program, code, codeName, headerNames == null ? 0 : headerNames.length, headerContentsPointer, headerNamesPointer);
         checkNVRTCError(error);

         // Compile the program
         error = nvrtcCompileProgram(program, compilationOptions == null ? 0 : compilationOptions.length, options);
         checkNVRTCError(error);

         // In case of error, print compilation log
         if (error != NVRTC_SUCCESS)
            printProgramLog(program, kernelName, Level.FATAL);

         // Get the PTX size
         error = nvrtcGetPTXSize(program, ptxSize);
         checkNVRTCError(error);

         // Get the PTX
         BytePointer ptx = new BytePointer(ptxSize.get());
         error = nvrtcGetPTX(program, ptx);
         checkNVRTCError(error);

         // Destroy the program; we don't need it anymore
         nvrtcDestroyProgram(program);

         return ptx;
      }
   }

   private static void printProgramLog(_nvrtcProgram program, String programName, Level logLevel)
   {
      int error;

      // Get the log size
      SizeTPointer logSize = new SizeTPointer(1L);
      error = nvrtcGetProgramLogSize(program, logSize);
      checkNVRTCError(error);
      // Get the log & print
      BytePointer log = new BytePointer(logSize.get());
      error = nvrtcGetProgramLog(program, log);
      checkNVRTCError(error);
      LogTools.log(logLevel, "Compilation of {} produced the following log:\n{}", programName, log.getString());

      logSize.close();
      log.close();
   }

   private static void checkNVRTCError(int errorCode)
   {
      if (errorCode == NVRTC_SUCCESS)
         return;

      try (BytePointer errorString = nvrtcGetErrorString(errorCode))
      {
         LogTools.error(errorString);
      }
   }

   private static void launchKernelFunction(CUstream_st stream, CUfunc_st function, dim3 gridSize, dim3 blockSize, int sharedMemorySize, Pointer... arguments)
   {
      PointerPointer<Pointer> argumentsPointer = arguments == null ? new PointerPointer<>() : new PointerPointer<>(arguments);
      int error = cuLaunchKernel(function,
                                 gridSize.x(),
                                 gridSize.y(),
                                 gridSize.z(),
                                 blockSize.x(),
                                 blockSize.y(),
                                 blockSize.z(),
                                 sharedMemorySize,
                                 stream,
                                 argumentsPointer,
                                 new Pointer());
      CUDATools.checkCUDAError(error);
      argumentsPointer.close();
   }

   public static void main(String[] args)
   {
      int error;

      CUstream_st stream = CUDAStreamManager.getStream();
      BytePointer ptx = compileKernelToPTX(KERNEL, "test.cu", null, null, null);

      // Create a module
      CUmod_st module = new CUmod_st();
      error = cuModuleLoadData(module, ptx);
      CUDATools.checkCUDAError(error);

      // Get the function
      CUfunc_st kernelFunction = new CUfunc_st();
      error = cuModuleGetFunction(kernelFunction, module, "test");
      CUDATools.checkCUDAError(error);

      dim3 blockSize = new dim3(10, 5, 1);
      dim3 gridSize = new dim3(1, 1, 1);

      // Allocate memory on device for output
      long outputSize = (long) blockSize.x() * blockSize.y() * blockSize.z() * gridSize.x() * gridSize.y() * gridSize.z();
      IntPointer deviceData = new IntPointer();
      cudaMallocAsync(deviceData, outputSize * deviceData.sizeof(), stream);

      // Launch the kernel
      launchKernelFunction(stream, kernelFunction, gridSize, blockSize, 0, deviceData);

      // Download device data to host
      IntPointer hostData = new IntPointer(outputSize * deviceData.sizeof());
      cudaMemcpyAsync(hostData, deviceData, outputSize * deviceData.sizeof(), cudaMemcpyDefault, stream);
      cudaFreeAsync(deviceData, stream);

      // Synchronize stream to ensure data is downloaded
      cudaStreamSynchronize(stream);

      // Print the results
      for (int y = 0; y < blockSize.y(); ++y)
      {
         for (int x = 0; x < blockSize.x(); ++x)
         {
            System.out.print(hostData.get((long) y * blockSize.y() + x) + " ");
         }
         System.out.println();
      }

      // Close everything
      CUDAStreamManager.releaseStream(stream);
      ptx.close();
      module.close();
      kernelFunction.close();
      blockSize.close();
      gridSize.close();
      deviceData.close();
      hostData.close();
   }
}
