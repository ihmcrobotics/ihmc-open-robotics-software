package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.cudaDeviceProp;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.cuda.global.nvjpeg;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.Pointer;
import us.ihmc.log.LogTools;

import java.net.URL;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.nvjpeg.NVJPEG_STATUS_SUCCESS;
import static org.bytedeco.cuda.global.nvrtc.NVRTC_SUCCESS;
import static org.bytedeco.cuda.global.nvrtc.nvrtcGetErrorString;

public class CUDATools
{
   public static boolean hasCUDA()
   {
      return hasLibrary(cudart.class);
   }

   public static boolean hasNVJPEG()
   {
      return hasLibrary(nvjpeg.class);
   }

   private static boolean hasLibrary(Class<?> libraryClass)
   {
      try
      {
         Loader.load(libraryClass);
         return true;
      }
      catch (Error libraryNotAvailable)
      {
         return false;
      }
   }

   public static int getCUDADeviceCount()
   {
      if (!hasCUDA())
         return 0;

      int[] devices = new int[1];
      checkCUDAError(cudaGetDeviceCount(devices));
      return devices[0];
   }

   public static boolean hasCUDADevice()
   {
      return getCUDADeviceCount() > 0;
   }

   /**
    * Get the name of a CUDA device
    * @param device the device number
    * @return the name of the device
    */
   public static String getDeviceName(int device)
   {
      try (IntPointer devicePointer = new IntPointer(1);
           cudaDeviceProp deviceProperties = new cudaDeviceProp())
      {
         devicePointer.put(device);
         cudaGetDeviceProperties(deviceProperties, devicePointer.get());
         return deviceProperties.name().getString();
      }
   }

   /**
    * Attempts to score GPUs from a baseline device name (e.g. the minimum could be "RTX 2080 Ti")
    *
    * @param deviceName        The device name to score
    * @param minimumDeviceName The device name to score against
    * @return true if deviceName is equal-to-or-better minimumDeviceName in terms of relative CUDA performance
    */
   public static boolean hasCUDADeviceOfAtLeast(String deviceName, String minimumDeviceName)
   {
      final double gpuSeriesExp = 0.8; // How much to weight the GPU series
      final int tiWeight = 5; // If the GPU is a Ti edition
      final int superWeight = 5; // If the GPU is a SUPER edition

      double minimumDeviceScore = 0.0;

      // Find the device number (e.g. 1080)
      Pattern numberPattern = Pattern.compile("\\d+");
      Matcher deviceMatcher = numberPattern.matcher(deviceName);
      Matcher minumumDeviceMatcher = numberPattern.matcher(minimumDeviceName);

      if (minumumDeviceMatcher.find())
      {
         String gpuModel = minumumDeviceMatcher.group();
         int gpuSeries = gpuModel.length() == 3 ? Integer.parseInt(gpuModel.substring(0, 1)) : Integer.parseInt(gpuModel.substring(0, 2));
         int gpuTier = gpuModel.length() == 3 ? Integer.parseInt(gpuModel.substring(1, 3)) : Integer.parseInt(gpuModel.substring(2, 4));
         boolean tiEdition = deviceName.toLowerCase().contains("ti");
         boolean superEdition = deviceName.toLowerCase().contains("super");
         minimumDeviceScore = Math.pow(gpuSeries, gpuSeriesExp) + gpuTier + (tiEdition ? tiWeight : 0) + (superEdition ? superWeight : 0);
      }

      if (deviceMatcher.find() && minimumDeviceScore > 0.0)
      {
         String gpuModel = deviceMatcher.group();
         int gpuSeries = gpuModel.length() == 3 ? Integer.parseInt(gpuModel.substring(0, 1)) : Integer.parseInt(gpuModel.substring(0, 2));
         int gpuTier = gpuModel.length() == 3 ? Integer.parseInt(gpuModel.substring(1, 3)) : Integer.parseInt(gpuModel.substring(2, 4));
         boolean tiEdition = deviceName.toLowerCase().contains("ti");
         boolean superEdition = deviceName.toLowerCase().contains("super");
         double score = Math.pow(gpuSeries, gpuSeriesExp) + gpuTier + (tiEdition ? tiWeight : 0) + (superEdition ? superWeight : 0);
         return score >= minimumDeviceScore;
      }

      return false;
   }

   /**
    * Each block of a CUDA kernel has a maximum number of threads it can run.
    * The block dimensions must not multiply to be greater than this number.
    * On older GPUs it is 512 threads and on newer models it is 1024.
    * @return The maximum number of threads per block of the device.
    */
   public static int getMaxThreadsPerBlock()
   {
      try (IntPointer device = new IntPointer(1);
           cudaDeviceProp deviceProperties = new cudaDeviceProp())
      {
         cudaGetDevice(device);
         cudaGetDeviceProperties(deviceProperties, device.get());
         return deviceProperties.maxThreadsPerBlock();
      }
   }

   /**
    * Allocate GPU memory for {@code elementCount} elements of the {@code cudaPointer}'s type.
    * <p>
    * THE TYPE OF THE PASSED IN POINTER MATTERS.
    * <p>
    * The size of the memory to allocate is determined by {@code cudaPointer.sizeof() * elementCount}.
    * As such, to allocate memory for 10 {@code floats}, the {@code cudaPointer} must be a {@link org.bytedeco.javacpp.FloatPointer}.
    *
    * @param cudaPointer  CUDA pointer that will point to the allocated GPU memory. THE TYPE OF THIS POINTER MATTERS.
    * @param elementCount Number of elements to allocate.
    * @param stream       The CUDA stream to use for asynchronous allocation.
    */
   public static void mallocAsync(Pointer cudaPointer, long elementCount, CUstream_st stream)
   {
      checkCUDAError(cudaMallocAsync(cudaPointer, cudaPointer.sizeof() * elementCount, stream));
   }

   /**
    * Allocate page-locked (pinned) host memory for {@code elementCount} elements of the {@code pointer}'s type.
    * <p>
    * Pinned memory enables faster asynchronous transfers between host and device. Use {@link #freeHost(Pointer)}
    * to release it. The size allocated is {@code pointer.sizeof() * elementCount}.
    *
    * @param pointer      Host pointer that will reference the allocated pinned memory. THE TYPE MATTERS.
    * @param elementCount Number of elements to allocate.
    */
   public static void mallocHost(Pointer pointer, long elementCount)
   {
      checkCUDAError(cudaMallocHost(pointer, pointer.sizeof() * elementCount));
   }

   /**
    * Free page-locked (pinned) host memory previously allocated with {@link #mallocHost(Pointer, long)}.
    *
    * @param pointer Host pointer to free.
    */
   public static void freeHost(Pointer pointer)
   {
      checkCUDAError(cudaFreeHost(pointer));
   }

   /**
    * Performs default memory copy (as in, {@link cudart#cudaMemcpyDefault}) from {@code source} to {@code destination}.
    * <p>
    * THE POINTER TYPE OF {@code source} MATTERS.
    * <p>
    * The number of bytes to copy is determined by {@code source.sizeof() * elementCount}.
    * As such, to copy 10 {@code floats}, the {@code source} pointer must be a {@link org.bytedeco.javacpp.FloatPointer}.
    *
    * @param destination  Pointer to which memory is copied.
    * @param source       Pointer from which memory is copied.
    * @param elementCount Number of elements to copy.
    * @param stream       The CUDA stream to use for asynchronous allocation.
    */
   public static void memcpyAsync(Pointer destination, Pointer source, long elementCount, CUstream_st stream)
   {
      checkCUDAError(cudaMemcpyAsync(destination, source, source.sizeof() * elementCount, cudaMemcpyDefault, stream));
   }

   /**
    * Helper function for CUDA error checking.
    *
    * @param errorCode The returned error code from a CUDA function.
    */
   public static void checkCUDAError(int errorCode)
   {
      if (errorCode != CUDA_SUCCESS)
      {
         try (BytePointer errorName = cudaGetErrorName(errorCode);
              BytePointer errorString = cudaGetErrorString(errorCode))
         {
            String errorMessage = String.format("CUDA Error (%s): %s", errorName.getString(), errorString.getString());
            LogTools.error(errorMessage);
         }
      }
   }

   public static void throwCUDAError(int errorCode) throws Exception
   {
      if (errorCode != CUDA_SUCCESS)
      {
         try (BytePointer errorName = cudaGetErrorName(errorCode);
              BytePointer errorString = cudaGetErrorString(errorCode))
         {
            String errorMessage = String.format("CUDA Error (%s): %s", errorName.getString(), errorString.getString());

            // Attempt to hint a solution
            if (errorCode == CUDA_ERROR_UNSUPPORTED_PTX_VERSION)
            {
               errorMessage += " [Tip] Your locally installed CUDA Toolkit version should match the NVIDIA Driver version. It is most likely that you need to update the NVIDIA Driver to resolve this error.";
            }

            throw new Exception("CUDA Error code: " + errorMessage);
         }
      }
   }

   /**
    * @return The URL to the Utils.cu file
    */
   public static URL getUtilsFile()
   {
      return CUDATools.class.getResource("Utils.cu");
   }

   /**
    * @return The URL to the PerceptionUtils.cu file
    */
   public static URL getPerceptionUtilsFile()
   {
      return CUDATools.class.getResource("PerceptionUtils.cu");
   }

   /**
    * Helper function for nvjpeg error checking.
    *
    * @param errorCode The returned error code from an nvJPEG function.
    */
   public static void checkNVJPEGError(int errorCode)
   {
      if (errorCode != NVJPEG_STATUS_SUCCESS)
      {
         String errorName = switch (errorCode)
         {
            case 1 -> "NVJPEG_STATUS_NOT_INITIALIZED";
            case 2 -> "NVJPEG_STATUS_INVALID_PARAMETER";
            case 3 -> "NVJPEG_STATUS_BAD_JPEG";
            case 4 -> "NVJPEG_STATUS_JPEG_NOT_SUPPORTED";
            case 5 -> "NVJPEG_STATUS_ALLOCATOR_FAILURE";
            case 6 -> "NVJPEG_STATUS_EXECUTION_FAILED";
            case 7 -> "NVJPEG_STATUS_ARCH_MISMATCH";
            case 8 -> "NVJPEG_STATUS_INTERNAL_ERROR";
            case 9 -> "NVJPEG_STATUS_IMPLEMENTATION_NOT_SUPPORTED";
            case 10 -> "NVJPEG_STATUS_INCOMPLETE_BITSTREAM";
            default -> "UNKNOWN";
         };

         String errorMessage = String.format("NVJPEG Error (%d): %s", errorCode, errorName);
         LogTools.error(errorMessage);
      }
   }

   public static void checkNVRTCError(int errorCode)
   {
      if (errorCode == NVRTC_SUCCESS)
         return;

      try (BytePointer errorString = nvrtcGetErrorString(errorCode))
      {
         String errorMessage = String.format("NVRTC Error (%d): %s", errorCode, errorString.getString());
         LogTools.error(errorMessage);
      }
   }
}
