package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerPointer;
import org.junit.jupiter.api.Test;

import static org.bytedeco.cuda.global.cudart.*;
import static org.junit.jupiter.api.Assertions.*;

public class CUDAProgramTest
{
   private static final String ADD_KERNEL = """
         extern "C"
         __global__
         void add(int a, int b, int * sum)
         {
            *sum = a + b;
         }
         """;

   @Test
   public void testSimpleKernel()
   {
      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();

      // Construct a program
      CUDAProgram additionProgram = new CUDAProgram("add.cu", ADD_KERNEL);

      // Load the kernel
      additionProgram.loadKernel("add");

      // Create host & device pointers
      IntPointer a = new IntPointer(1L).put(3);
      IntPointer b = new IntPointer(1L).put(7);
      IntPointer sum = new IntPointer(1L);
      PointerPointer<IntPointer> sumPointer = new PointerPointer<>(1L);
      sumPointer.put(sum);

      // Run the kernel
      additionProgram.runKernel(stream, "add", new dim3(1, 1, 1), new dim3(1, 1, 1), 0, a, b, sumPointer);
      cudaStreamSynchronize(stream);

      // Ensure we got the correct result!
      assertEquals(10, sum.get());

      // Free memory
      a.close();
      b.close();
      sum.close();

      additionProgram.destroy();

      CUDAStreamManager.releaseStream(stream);
   }
}
