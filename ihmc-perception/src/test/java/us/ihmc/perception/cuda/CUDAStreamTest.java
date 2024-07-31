package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class CUDAStreamTest
{
   @Test
   public void testCreateReleaseCreate()
   {
      CUstream_st stream = CUDAStream.getStream();
      assertEquals(1, CUDAStream.getReferenceCount());

      CUDAStream.releaseStream(stream);
      assertEquals(0, CUDAStream.getReferenceCount());

      CUstream_st newStream = CUDAStream.getStream();
      assertEquals(1, CUDAStream.getReferenceCount());

      CUDAStream.releaseStream(newStream);
      assertEquals(0, CUDAStream.getReferenceCount());
   }
}
