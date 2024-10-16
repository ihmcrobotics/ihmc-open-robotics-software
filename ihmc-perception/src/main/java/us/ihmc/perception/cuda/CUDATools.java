package us.ihmc.perception.cuda;

import org.bytedeco.cuda.global.cudart;
import org.bytedeco.cuda.global.nvcomp;
import org.bytedeco.cuda.global.nvjpeg;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Loader;
import us.ihmc.log.LogTools;

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

   public static boolean hasNVCOMP()
   {
      return hasLibrary(nvcomp.class);
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
    * Helper function for CUDA error checking.
    * @param errorCode The returned error code from a CUDA function.
    */
   public static void checkCUDAError(int errorCode)
   {
      if (errorCode != CUDA_SUCCESS)
      {
         try (BytePointer errorName = cudaGetErrorName(errorCode);
              BytePointer errorString = cudaGetErrorString(errorCode))
         {
            LogTools.error("CUDA Error ({}): {}", errorName.getString(), errorString.getString());
         }
      }
   }

   /**
    * Helper function for nvjpeg error checking.
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
         LogTools.error("NVJPEG Error ({}): {}", errorCode, errorName);
      }
   }

   public static void checkNVRTCError(int errorCode)
   {
      if (errorCode == NVRTC_SUCCESS)
         return;

      try (BytePointer errorString = nvrtcGetErrorString(errorCode))
      {
         LogTools.error("NVRTC error: {}", errorString.getString());
      }
   }
}
