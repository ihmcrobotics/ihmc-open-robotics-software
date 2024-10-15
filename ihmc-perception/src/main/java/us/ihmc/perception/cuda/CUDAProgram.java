package us.ihmc.perception.cuda;

import org.apache.logging.log4j.Level;
import org.bytedeco.cuda.cudart.CUfunc_st;
import org.bytedeco.cuda.cudart.CUmod_st;
import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.nvrtc._nvrtcProgram;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.javacpp.SizeTPointer;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.nvrtc.*;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;
import static us.ihmc.perception.cuda.CUDATools.checkNVRTCError;

class CUDAProgram
{
   private final Map<String, CUfunc_st> kernels = new HashMap<>();

   private final CUmod_st module = new CUmod_st();

   private int error;

   public CUDAProgram(Path programPath, Path... headerPaths)
   {
      try
      {
         String programName = programPath.getFileName().toString();
         String programContents = new String(Files.readAllBytes(programPath));

         String[] headerNames = null;
         String[] headerContents = null;

         if (headerPaths != null && headerPaths.length > 0)
         {
            headerNames = new String[headerPaths.length];
            headerContents = new String[headerPaths.length];

            for (int i = 0; i < headerPaths.length; i++)
            {
               headerNames[i] = headerPaths[i].getFileName().toString();
               headerContents[i] = new String(Files.readAllBytes(headerPaths[i]));
            }
         }

         initialize(programName, programContents, headerNames, headerContents);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public CUDAProgram(String programName, String programCode)
   {
      this(programName, programCode, null, null);
   }

   public CUDAProgram(String programName, String programCode, String[] headerNames, String[] headerContents)
   {
      initialize(programName, programCode, headerNames, headerContents);
   }

   public void loadKernel(String kernelName)
   {
      // Load the kernel from module
      CUfunc_st kernel = new CUfunc_st();
      error = cuModuleGetFunction(kernel, module, kernelName);
      checkCUDAError(error);
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

      cuModuleUnload(module);
      module.close();
   }

   private void initialize(String programName, String programCode, String[] headerNames, String[] headerContents)
   {
      // Compile the program
      _nvrtcProgram compiledProgram = new _nvrtcProgram();
      compileProgram(programName, programCode, headerNames, headerContents, null, compiledProgram);

      // Get the program's PTX size
      SizeTPointer ptxSize = new SizeTPointer(1L);
      error = nvrtcGetPTXSize(compiledProgram, ptxSize);
      checkNVRTCError(error);

      // Get the PTX
      BytePointer ptx = new BytePointer(ptxSize.get());
      error = nvrtcGetPTX(compiledProgram, ptx);
      checkNVRTCError(error);

      // Load the module using the PTX
      error = cuModuleLoadData(module, ptx);
      checkNVRTCError(error);

      // Release stuff
      nvrtcDestroyProgram(compiledProgram);
      compiledProgram.close();
      ptxSize.close();
      ptx.close();
   }

   private static void compileProgram(String programName,
                                      String programCode,
                                      String[] headerNames,
                                      String[] headerContents,
                                      String[] compilationOptions,
                                      _nvrtcProgram compiledProgram)
   {
      int error;
      try (BytePointer programNamePointer = new BytePointer(programName);
           BytePointer programCodePointer = new BytePointer(programCode);
           PointerPointer<BytePointer> headerContentsPointer = headerContents == null ? null : new PointerPointer<>(headerContents);
           PointerPointer<BytePointer> headerNamesPointer = headerNames == null ? null : new PointerPointer<>(headerNames);
           PointerPointer<BytePointer> options = compilationOptions == null ? null : new PointerPointer<>(compilationOptions))
      {
         // Create the program
         error = nvrtcCreateProgram(compiledProgram,
                                    programCodePointer,
                                    programNamePointer,
                                    headerNames == null ? 0 : headerNames.length,
                                    headerContentsPointer,
                                    headerNamesPointer);
         checkNVRTCError(error);

         // Compile the program
         error = nvrtcCompileProgram(compiledProgram, compilationOptions == null ? 0 : compilationOptions.length, options);
         checkNVRTCError(error);

         // In case of error, print compilation log
         if (error != NVRTC_SUCCESS)
            printProgramLog(compiledProgram, programName, Level.FATAL);
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
                                 new PointerPointer<>());

      CUDATools.checkCUDAError(error);
      argumentsPointer.close();
   }
}
