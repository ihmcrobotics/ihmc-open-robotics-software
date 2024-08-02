package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.global.cudart;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

/**
 * Limits the number of CUDA streams created to (ideally) the number of device connections.
 * It seems that having more streams than number of device connections is not beneficial.
 * See: <a href="https://forums.developer.nvidia.com/t/how-many-streams-maximum-number-of-streams/6571/17">Nvidia Forum Discussion</a>
 */
public class CUDAStreamManager
{
   // TODO: Find way to get value of CUDA_DEVICE_MAX_CONNECTIONS variable
   public static final int MAX_CUDA_STREAMS = 8;

   private static final List<ReferencedCUDAStream> streams = new ArrayList<>(MAX_CUDA_STREAMS);
   private static int streamsGotten = 0;

   public static synchronized CUstream_st getStream()
   {
      if (streams.size() < MAX_CUDA_STREAMS)
      {
         ReferencedCUDAStream stream = new ReferencedCUDAStream();
         streams.add(stream);
      }

      return streams.get(streamsGotten++ % streams.size()).get();
   }

   @SuppressWarnings("SuspiciousMethodCalls")
   public static synchronized void releaseStream(CUstream_st stream)
   {
      int index = streams.indexOf(stream);
      if (index < 0)
         throw new IllegalArgumentException("Attempting to release an independent stream not managed by " + CUDAStreamManager.class.getSimpleName());

      if (streams.get(index).release())
         streams.remove(index);
   }

   /* package-private */ static synchronized int getNumberOfActiveStreams()
   {
      return streams.size();
   }

   private static class ReferencedCUDAStream extends CUstream_st
   {
      private final AtomicLong references = new AtomicLong(0L);
      private final AtomicBoolean destroyed = new AtomicBoolean(false);

      private ReferencedCUDAStream()
      {
         checkCUDAError(cudart.cudaStreamCreate(this));
      }

      /**
       * Get a reference to this stream
       * @return this.
       */
      private ReferencedCUDAStream get()
      {
         if (destroyed.get())
            return null;

         references.incrementAndGet();
         return this;
      }

      /**
       * Release a reference to this stream
       * @return true if the stream is destroyed, otherwise false
       */
      private boolean release()
      {
         if (references.decrementAndGet() == 0L)
         {
            checkCUDAError(cudart.cudaStreamSynchronize(this));
            checkCUDAError(cudart.cudaStreamDestroy(this));
            close();

            return true;
         }

         return false;
      }
   }
}
