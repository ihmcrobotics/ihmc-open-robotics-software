package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.junit.jupiter.api.Test;

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
}
