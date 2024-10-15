package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerPointer;
import org.junit.jupiter.api.Test;

import java.net.URISyntaxException;
import java.nio.file.Path;
import java.util.Objects;

import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;
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

   private static final String KERNEL_HEADER = """
         __constant__ int a = 3;
         __constant__ int b = 7;
         """;

   private static final String KERNEL_WITH_HEADER = """
         #include "test_values.cuh"

         extern "C"
         __global__
         void add(int * sum)
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
      sumPointer.close();

      additionProgram.destroy();

      CUDAStreamManager.releaseStream(stream);
   }

   @Test
   public void testKernelWithHeader()
   {
      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();

      // Construct a program
      String[] headerName = {"test_values.cuh"};
      String[] headerContents = {KERNEL_HEADER};

      CUDAProgram additionProgram = new CUDAProgram("add_header.cu", KERNEL_WITH_HEADER, headerName, headerContents);

      // Load the kernel
      additionProgram.loadKernel("add");

      // Create pointer
      IntPointer sum = new IntPointer(1L);
      PointerPointer<IntPointer> sumPointer = new PointerPointer<>(1L);
      sumPointer.put(sum);

      // Run the kernel
      additionProgram.runKernel(stream, "add", new dim3(1, 1, 1), new dim3(1, 1, 1), 0, sumPointer);
      cudaStreamSynchronize(stream);

      // Ensure we got the correct result!
      assertEquals(10, sum.get());

      // Free memory
      sum.close();
      sumPointer.close();

      additionProgram.destroy();

      CUDAStreamManager.releaseStream(stream);
   }

   @Test
   public void testLoadingKernelFromFile() throws URISyntaxException
   {
      // Get a stream
      CUstream_st stream = CUDAStreamManager.getStream();

      // Create a CUDA program with files
      Path kernelPath = Path.of(Objects.requireNonNull(getClass().getResource("test_add_values.cu")).toURI());
      Path headerPath = Path.of(Objects.requireNonNull(getClass().getResource("test_values.cuh")).toURI());
      CUDAProgram program = new CUDAProgram(kernelPath, headerPath);

      // Load the kernels
      program.loadKernel("add");
      program.loadKernel("subtract");

      // Create pointer
      IntPointer sum = new IntPointer(1L);
      PointerPointer<IntPointer> sumPointer = new PointerPointer<>(1L);
      sumPointer.put(sum);

      IntPointer difference = new IntPointer(1L);
      PointerPointer<IntPointer> differencePointer = new PointerPointer<>(1L);
      differencePointer.put(difference);

      // Run the kernels
      program.runKernel(stream, "add", new dim3(), new dim3(), 0, sumPointer);
      program.runKernel(stream, "subtract", new dim3(), new dim3(), 0, differencePointer);
      cudaStreamSynchronize(stream);

      // Ensure we got the correct result!
      assertEquals(10, sum.get());
      assertEquals(4, difference.get());

      // Free memory
      sum.close();
      sumPointer.close();
      difference.close();
      differencePointer.close();

      program.destroy();

      CUDAStreamManager.releaseStream(stream);
   }
}
