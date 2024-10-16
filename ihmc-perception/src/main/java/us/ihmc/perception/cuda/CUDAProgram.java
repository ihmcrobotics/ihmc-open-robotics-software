package us.ihmc.perception.cuda;

import org.apache.logging.log4j.Level;
import org.bytedeco.cuda.cudart.CUfunc_st;
import org.bytedeco.cuda.cudart.CUmod_st;
import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.cudaDeviceProp;
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

public class CUDAProgram
{
   private static final String[] DEFAULT_OPTIONS = {"-arch=" + getComputeVersion(),  // Target fairly recent GPU architecture
                                                    "--dopt=on",                     // Optimize code
                                                    "-G"};                           // More code optimization

   private final CUmod_st module = new CUmod_st();
   private final Map<String, CUfunc_st> kernels = new HashMap<>();

   private int error;

   /**
    * Construct a {@link CUDAProgram} with default compilation options
    *
    * @param programPath {@link Path} to the .cu file.
    * @param headerPaths {@link Path}s to the header files included (with {@code #include}) in the .cu file.
    */
   public CUDAProgram(Path programPath, Path... headerPaths)
   {
      this(programPath, headerPaths, DEFAULT_OPTIONS);
   }

   /**
    * Construct a {@link CUDAProgram} specifying the path to the .cu file, paths to the header files, and compilation options.
    *
    * @param programPath        {@link Path} to the .cu file.
    * @param headerPaths        {@link Path}s to the header files included (with {@code #include}) in the .cu file.
    * @param compilationOptions List of compilation options
    *                           (You can see the available options <a
    *                           href="https://docs.nvidia.com/cuda/nvrtc/index.html#supported-compile-options">here</a>)
    */
   public CUDAProgram(Path programPath, Path[] headerPaths, String... compilationOptions)
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

         initialize(programName, programContents, headerNames, headerContents, compilationOptions);
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

   /**
    * Construct a {@link CUDAProgram} with default compilation options.
    *
    * @param programName    The name of the program (usually the file name with a .cu extension).
    * @param programCode    The program code (i.e. the contents of the .cu file).
    * @param headerNames    List of header names included (with {@code #include}) in the code.
    * @param headerContents Contents of the headers included in the code.
    */
   public CUDAProgram(String programName, String programCode, String[] headerNames, String[] headerContents)
   {
      this(programName, programCode, headerNames, headerContents, DEFAULT_OPTIONS);
   }

   /**
    * Construct a {@link CUDAProgram} specifying the name, code, header names, header contents, and compilation options.
    *
    * @param programName        The name of the program (usually the file name with a .cu extension).
    * @param programCode        The program code (i.e. the contents of the .cu file).
    * @param headerNames        List of header names included (with {@code #include}) in the code.
    * @param headerContents     Contents of the headers included in the code.
    * @param compilationOptions List of compilation options
    *                           (You can see the available options <a
    *                           href="https://docs.nvidia.com/cuda/nvrtc/index.html#supported-compile-options">here</a>)
    */
   public CUDAProgram(String programName, String programCode, String[] headerNames, String[] headerContents, String... compilationOptions)
   {
      initialize(programName, programCode, headerNames, headerContents, compilationOptions);
   }

   public CUDAKernelHandle loadKernel(String kernelName)
   {
      // Load the kernel from module
      CUfunc_st kernel = new CUfunc_st();
      error = cuModuleGetFunction(kernel, module, kernelName);
      checkCUDAError(error);
      kernels.put(kernelName, kernel);

      CUDAKernelHandle kernelHandle = new CUDAKernelHandle(kernelName);
      // TODO figure out how to load the grid size and block size from the kernel definition when the kernel is loaded. Then set them here.

      return kernelHandle;
   }

   public void runKernel(CUstream_st stream, CUDAKernelHandle kernelHandle, int sharedMemorySize, Pointer... arguments)
   {
      runKernel(stream, kernelHandle.getName(), kernelHandle.getGridSize(), kernelHandle.getBlockSize(), sharedMemorySize, arguments);
   }

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

   private void initialize(String programName, String programCode, String[] headerNames, String[] headerContents, String[] compilationOptions)
   {
      // Compile the program
      _nvrtcProgram compiledProgram = new _nvrtcProgram();
      compileProgram(programName, programCode, headerNames, headerContents, compilationOptions, compiledProgram);

      // Get the program's PTX size
      SizeTPointer ptxSize = new SizeTPointer(1L);
      error = nvrtcGetPTXSize(compiledProgram, ptxSize);
      checkNVRTCError(error);

      // Get the PTX
      BytePointer ptx = new BytePointer(ptxSize.get());
      error = nvrtcGetPTX(compiledProgram, ptx);
      checkNVRTCError(error);

      LogTools.debug("The following PTX was generated for {}:\n{}", programName, ptx.getString());

      // Load the module using the PTX
      error = cuModuleLoadData(module, ptx);
      checkNVRTCError(error);

      // Release stuff
      nvrtcDestroyProgram(compiledProgram);
      compiledProgram.close();
      ptxSize.close();
      ptx.close();
   }

   /**
    * Compiles a CUDA program to an {@code _nvrtcProgram}.
    * @param programName [IN] The name of the program (usually the file name with a .cu extension).
    * @param programCode [IN] The program code (i.e. the contents of the .cu file).
    * @param headerNames [IN] List of header names included (with {@code #include}) in the code.
    * @param headerContents [IN] Contents of the headers included in the code.
    * @param compilationOptions [IN] List of compilation options
    *                           (You can see the available options <a href="https://docs.nvidia.com/cuda/nvrtc/index.html#supported-compile-options">here</a>)
    * @param compiledProgram [OUT] The compiled program.
    */
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

   /**
    * Retrieves and logs the compilation log of the compiled {@code program}.
    * @param program The compiled program.
    * @param programName Name of the compiled program.
    * @param logLevel Level at which to output the log.
    */
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

   /**
    * Launches a CUDA kernel.
    * @param stream CUDA stream on which the kernel will be synchronized.
    * @param function The kernel function to launch.
    * @param gridSize Grid size of the kernel execution.
    * @param blockSize Block size of the kernel execution. Should not exceed maximum block size of the device.
    * @param sharedMemorySize Size, in byte, of the memory shared by threads in each block.
    * @param arguments List of arguments (parameters) being passed into the kernel.
    */
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

   /**
    * @return A fairly recent compute version available on the device.
    */
   private static String getComputeVersion()
   {
      cudaDeviceProp deviceProperties = new cudaDeviceProp();
      cudaGetDeviceProperties(deviceProperties, 0);
      int majorVersion = deviceProperties.major();
      int minorVersion = deviceProperties.minor();

      String computeVersion = "compute_" + majorVersion + "0";

      LogTools.debug("Found CUDA architecture {}.{}. Targeting {} for CUDA kernel compilation.", majorVersion, minorVersion, computeVersion);

      deviceProperties.close();

      return computeVersion;
   }
}
