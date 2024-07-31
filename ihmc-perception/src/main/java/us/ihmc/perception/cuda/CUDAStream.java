package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.global.cudart;

import java.util.concurrent.atomic.AtomicLong;

import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

/**
 * Wrapper for CUstream_st to simplify access to streams
 */
public class CUDAStream
{
   private static CUstream_st stream = null;
   private static final AtomicLong referenceCount = new AtomicLong(0L);

   public static synchronized CUstream_st getStream()
   {
      if (stream == null || stream.referenceCount() <= 0)
      {
         stream = new CUstream_st();
         checkCUDAError(cudart.cudaStreamCreate(stream));
      }

      referenceCount.getAndIncrement();

      return stream;
   }

   public static synchronized void releaseStream(CUstream_st streamToRelease)
   {
      if (streamToRelease != stream)
         throw new IllegalArgumentException("Attempting to release another stream");

      if (referenceCount.decrementAndGet() == 0)
      {
         checkCUDAError(cudart.cudaStreamSynchronize(stream));
         checkCUDAError(cudart.cudaStreamDestroy(stream));
         stream.close();
      }
   }

   public static long getReferenceCount()
   {
      return referenceCount.get();
   }
}
