package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;

import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

public class CUDAStreamManagerTest
{
   @Test
   public void testCreateReleaseCreate()
   {
      CUstream_st firstStream = CUDAStreamManager.getStream();
      assertFalse(firstStream.isNull());

      for (int i = 1; i < CUDAStreamManager.MAX_CUDA_STREAMS; ++i)
      {
         CUDAStreamManager.getStream();
      }

      CUstream_st stream = CUDAStreamManager.getStream();
      assertEquals(firstStream, stream);
   }

   @Test
   public void testConcurrency()
   {
      Random random = new Random(0L);
      AtomicBoolean exceptionThrown = new AtomicBoolean(false);

      for (int i = 0; i < CUDAStreamManager.MAX_CUDA_STREAMS; ++i)
      {
         new Thread(() ->
         {
            ThreadTools.sleep(random.nextLong(10L));

            try
            {
               CUDAStreamManager.getStream();
            }
            catch (Exception e)
            {
               exceptionThrown.set(true);
            }
         }).start();
      }

      assertFalse(exceptionThrown.get());
   }
}
