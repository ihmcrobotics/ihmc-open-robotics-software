package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.IntPointer;
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

   @SuppressWarnings("all")
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
      IntPointer a = new IntPointer(1L);
      a.put(3);
      IntPointer aDevice = new IntPointer();
      IntPointer b = new IntPointer(1L);
      b.put(7);
      IntPointer bDevice = new IntPointer();
      IntPointer sum = new IntPointer(1L);
      IntPointer sumDevice = new IntPointer();

      // Allocate device memory & copy data to device
      cudaMallocAsync(aDevice, 1L * a.sizeof(), stream);
      cudaMemcpyAsync(aDevice, a, 1L * a.sizeof(), cudaMemcpyDefault, stream);
      cudaMallocAsync(bDevice, 1L * b.sizeof(), stream);
      cudaMemcpyAsync(bDevice, b, 1L * b.sizeof(), cudaMemcpyDefault, stream);
      cudaMallocAsync(sumDevice, 1L * sum.sizeof(), stream);

      // Run the kernel
      additionProgram.runKernel(stream, "add", new dim3(1, 1, 1), new dim3(1, 1, 1), 0, aDevice, bDevice, sumDevice);
      cudaStreamSynchronize(stream);
      // Download the result data
      cudaMemcpyAsync(sum, sumDevice, 1L * sumDevice.sizeof(), cudaMemcpyDefault, stream);
      cudaStreamSynchronize(stream);
      // Ensure we got the correct result!
      assertEquals(10, sum.get());

      // Free memory
      cudaFree(aDevice);
      aDevice.close();
      cudaFree(bDevice);
      bDevice.close();
      cudaFree(sumDevice);
      sumDevice.close();

      a.close();
      b.close();
      sum.close();

      additionProgram.destroy();

      CUDAStreamManager.releaseStream(stream);
   }
}
