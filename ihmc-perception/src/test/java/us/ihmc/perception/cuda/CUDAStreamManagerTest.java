package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAStreamManager.CUDAStreamManagerInternal;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

public class CUDAStreamManagerTest
{
   private static boolean cudaWarningPrinted = false;

   @Test
   public void testCreateAndRelease()
   {
      CUDAStreamManagerInternal cudaStreamManager = new CUDAStreamManagerInternal();

      List<CUstream_st> streams = new ArrayList<>();
      for (int i = 0; i < CUDAStreamManager.MAX_CUDA_STREAMS; ++i)
      {
         CUstream_st stream = cudaStreamManager.getStream();
         assertNotNull(stream);
         streams.add(stream);
      }

      assertEquals(CUDAStreamManager.MAX_CUDA_STREAMS, cudaStreamManager.getNumberOfActiveStreams());

      for (int i = 0; i < CUDAStreamManager.MAX_CUDA_STREAMS; ++i)
      {
         CUstream_st stream = cudaStreamManager.getStream();
         assertEquals(streams.get(i), stream);
         streams.add(stream);
      }

      assertEquals(CUDAStreamManager.MAX_CUDA_STREAMS, cudaStreamManager.getNumberOfActiveStreams());

      for (CUstream_st stream : streams)
      {
         assertDoesNotThrow(() -> cudaStreamManager.releaseStream(stream));
      }

      assertEquals(0, cudaStreamManager.getNumberOfActiveStreams());
   }

   @Test
   public void testConcurrency() throws InterruptedException
   {
      CUDAStreamManagerInternal cudaStreamManager = new CUDAStreamManagerInternal();

      Random random = new Random(0L);
      AtomicBoolean exceptionThrown = new AtomicBoolean(false);

      int numRuns = 100;
      Thread[] threads = new Thread[numRuns];

      for (int i = 0; i < numRuns; ++i)
      {
         threads[i] = new Thread(() ->
         {
            try
            {
               ThreadTools.sleep(random.nextLong(10L));
               CUstream_st stream = cudaStreamManager.getStream();
               ThreadTools.sleep(random.nextLong(10L));
               cudaStreamManager.releaseStream(stream);
            }
            catch (UnsatisfiedLinkError error)
            {
               if (!cudaWarningPrinted)
               {
                  LogTools.error("Could not load CUDA library");
                  cudaWarningPrinted = true;
               }
            }
            catch (Exception exception)
            {
               LogTools.error("{}\n{}", exception.getMessage(), exception.getStackTrace());
               exceptionThrown.set(true);
            }
         }, this.getClass().getSimpleName() + "Thread" + i);
      }

      for (Thread thread : threads)
         thread.start();

      for (Thread thread : threads)
         thread.join();

      assertFalse(exceptionThrown.get());
      assertEquals(0, cudaStreamManager.getNumberOfActiveStreams());
   }
}
