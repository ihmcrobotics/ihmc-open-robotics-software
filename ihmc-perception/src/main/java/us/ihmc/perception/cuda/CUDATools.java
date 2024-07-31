package us.ihmc.perception.cuda;

import org.bytedeco.javacpp.BytePointer;
import us.ihmc.log.LogTools;

import static org.bytedeco.cuda.global.cudart.*;

public class CUDATools
{
   public static void checkError(int errorCode)
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
}
