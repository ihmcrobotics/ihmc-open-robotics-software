package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.global.cudart;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

public class CUDAStreamManager
{
   static
   {
      Runtime.getRuntime().addShutdownHook(new Thread(CUDAStreamManager::destroy, "CUDAStreamDestruction"));
   }

   // TODO: Find way to get value of CUDA_DEVICE_MAX_CONNECTIONS variable
   public static final int MAX_CUDA_STREAMS = 8;

   private static final List<CUstream_st> streams = new ArrayList<>(MAX_CUDA_STREAMS);
   private static int streamsGotten = 0;

   public static synchronized CUstream_st getStream()
   {
      if (streams.size() < MAX_CUDA_STREAMS)
      {
         CUstream_st stream = new CUstream_st();
         checkCUDAError(cudart.cudaStreamCreate(stream));
         streams.add(stream);
      }

      return streams.get(streamsGotten++ % MAX_CUDA_STREAMS);
   }

   private static void destroy()
   {
      System.out.println("Destroying CUDA streams");
      for (CUstream_st stream : streams)
      {
         checkCUDAError(cudart.cudaStreamSynchronize(stream));
         checkCUDAError(cudart.cudaStreamDestroy(stream));
         stream.close();
      }
      System.out.println("Destroyed CUDA streams");
   }
}
