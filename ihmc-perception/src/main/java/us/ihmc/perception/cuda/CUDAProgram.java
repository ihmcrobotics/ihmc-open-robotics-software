package us.ihmc.perception.cuda;

import org.apache.logging.log4j.Level;
import org.bytedeco.cuda.cudart.CUfunc_st;
import org.bytedeco.cuda.cudart.CUmod_st;
import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.nvrtc._nvrtcProgram;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.javacpp.SizeTPointer;
import us.ihmc.log.LogTools;

import java.util.HashMap;
import java.util.Map;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.nvrtc.*;
import static org.bytedeco.cuda.global.nvrtc.nvrtcDestroyProgram;
import static us.ihmc.perception.cuda.CUDATools.checkNVRTCError;

class CUDAProgram
{
   private final Map<String, CUfunc_st> kernels = new HashMap<>();

   private final CUmod_st module = new CUmod_st();

   private int error;

//   public CUDAProgram(String programFile, String... headerFiles)
//   {
//      // Call constructor with default options
//   }
//
//   public CUDAProgram(String programFile, String[] compilationOptions, String... headerFiles)
//   {
//      // Create PTX, module
//   }

   public CUDAProgram(String programName, String programCode)
   {
      BytePointer ptx = compileKernelToPTX(programName, programCode, null, null, null);
      error = cuModuleLoadData(module, ptx);
      checkNVRTCError(error);
      ptx.close();
   }

   public void loadKernel(String kernelName)
   {
      // Load the kernel from module
      CUfunc_st kernel = new CUfunc_st();
      error = cuModuleGetFunction(kernel, module, kernelName);
      checkNVRTCError(error);
      kernels.put(kernelName, kernel);
   }

//   public void runKernel(String kernelName, Mat image, Pointer... arguments)
//   {
//      GpuMat gpuMat = new GpuMat()
//      gpuMat.upload(image)
//      runKernel(...)
//   }
//
//   public void runKernel(String kernelName, GpuMat image, Pointer... arguments)
//   {
//      runKernel2D(kernelName, image.getWidth, image.getHeight, /*cuda pointer*/ image.data(), arguments);
//   }
//
//   public void runKernel2D(String kernelName, int dimX, int dimY, Pointer... arguments)
//   {
//      runKernel(kernelName, /* dims */);
//   }

   // TODO: see if shared memory size can be found programmatically
   public void runKernel(CUstream_st stream, String kernelName, dim3 gridSize, dim3 blockSize, int sharedMemorySize, Pointer... arguments)
   {
      if (!kernels.containsKey(kernelName))
         throw new IllegalStateException("Kernel (" + kernelName + ") requested has not been loaded yet.");

      // Run kernel
      CUfunc_st kernel = kernels.get(kernelName);
      launchKernelFunction(stream, kernel, gridSize, blockSize, sharedMemorySize, arguments);
   }

   public void destroy()
   {
      for (CUfunc_st kernel : kernels.values())
         kernel.close();

      module.close();
   }

   private static BytePointer compileKernelToPTX(String programName,
                                                 String programCode,
                                                 String[] headerNames,
                                                 String[] headerContents,
                                                 String[] compilationOptions)
   {
      int error;
      try (BytePointer programNamePointer = new BytePointer(programName);
           BytePointer programCodePointer = new BytePointer(programCode);
           PointerPointer<BytePointer> headerNamesPointer = headerNames == null ? null : new PointerPointer<>(headerNames);
           PointerPointer<BytePointer> headerContentsPointer = headerContents == null ? null : new PointerPointer<>(headerContents);
           PointerPointer<BytePointer> options = compilationOptions == null ? null : new PointerPointer<>(compilationOptions);

           _nvrtcProgram program = new _nvrtcProgram();
           SizeTPointer ptxSize = new SizeTPointer(1L))
      {
         // Create the program
         error = nvrtcCreateProgram(program, programCodePointer, programNamePointer, headerNames == null ? 0 : headerNames.length, headerContentsPointer, headerNamesPointer);
         checkNVRTCError(error);

         // Compile the program
         error = nvrtcCompileProgram(program, compilationOptions == null ? 0 : compilationOptions.length, options);
         checkNVRTCError(error);

         // In case of error, print compilation log
         if (error != NVRTC_SUCCESS)
            printProgramLog(program, programName, Level.FATAL);

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

   private static void launchKernelFunction(CUstream_st stream, CUfunc_st function, dim3 gridSize, dim3 blockSize, int sharedMemorySize, Pointer... arguments)
   {
      LongPointer argumentsPointer = arguments == null ? new LongPointer() : new LongPointer(arguments.length);
      if (arguments != null)
      {
         for (int i = 0; i < arguments.length; i++)
         {
            argumentsPointer.put(i, arguments[i].address());
         }
      }
      PointerPointer<LongPointer> argumentsPointerPointer = new PointerPointer<>(argumentsPointer);

      //Pointer argumentsPointer = Pointer.malloc(arguments == null ? 0L : (long) arguments.length * Long.BYTES);
//      PointerPointer<Pointer> argumentsPointer = new PointerPointer<>(arguments);

//      PointerPointer<Pointer> argumentsPointer = new PointerPointer<>(arguments);
//      PointerPointer<PointerPointer<Pointer>> argumentsPointerPointer = new PointerPointer<>(1L);
//      argumentsPointerPointer.put(argumentsPointer);

//      if (arguments != null)
//      {
//         for (int i = 0; i < arguments.length; i++)
//         {
//            System.out.println("Running " + i);
//            argumentsPointer.position(i).put(arguments[i]);
//         }
//         argumentsPointer.position(0);
//      }

      int error = cuLaunchKernel(function,
                                 gridSize.x(),
                                 gridSize.y(),
                                 gridSize.z(),
                                 blockSize.x(),
                                 blockSize.y(),
                                 blockSize.z(),
                                 sharedMemorySize,
                                 stream,
                                 argumentsPointerPointer,
                                 new Pointer());

      CUDATools.checkCUDAError(error);
      argumentsPointer.close();
   }
}
