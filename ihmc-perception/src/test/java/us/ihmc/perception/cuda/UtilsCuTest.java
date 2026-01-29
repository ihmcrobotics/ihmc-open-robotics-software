package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.IntPointer;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.Objects;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;
import static org.junit.jupiter.api.Assertions.*;

public class UtilsCuTest
{
   private static final String REDUCE_ADD_TEST_PROGRAM = """
         #include "Utils.cu"
         
         extern "C"
         __global__ void testReduceAdd(unsigned int value, unsigned int* result)
         {
            extern __shared__ unsigned int sharedArray[];
            Utils::reduceAdd(value, sharedArray, result);
         }
         """;

   private static final String[] UTILS_CU_HEADER = new String[]{"Utils.cu"};
   private static final String[] UTILS_CU_CONTENTS = new String[1];

   @BeforeAll
   public static void loadUtilsCu() throws IOException
   {
      URL utilsURL = CUDATools.class.getResource("Utils.cu");
      Objects.requireNonNull(utilsURL);

      InputStream programContentsStream = utilsURL.openStream();
      UTILS_CU_CONTENTS[0] = new String(programContentsStream.readAllBytes());
      programContentsStream.close();
   }

   @Test
   public void testReduceAdd() throws Exception
   {
      // Full blocks of 1024 threads
      testReduceAdd(1024, 1024, 32, 32, 1);
      testReduceAdd(1024, 1024, 32, 32, 2);

      // Blocks of 512 threads
      testReduceAdd(2048, 1024, 32, 16, 1);
      testReduceAdd(2048, 1024, 32, 16, 2);

      // Blocks of 256 threads
      testReduceAdd(2048, 2048, 16, 16, 1);
      testReduceAdd(2048, 2048, 16, 16, 2);

      // Blocks of 128 threads
      testReduceAdd(4096, 2048, 8, 16, 1);
      testReduceAdd(4096, 2048, 8, 16, 2);

      // Blocks of 64 threads
      testReduceAdd(4096, 4096, 8, 8, 1);
      testReduceAdd(4096, 4096, 8, 8, 2);

      // Blocks of 32 threads
      testReduceAdd(4096, 4096, 16, 2, 1);
      testReduceAdd(4096, 4096, 8, 4, 2);

      // Blocks of 16 threads
      testReduceAdd(4096, 4096, 4, 4, 1);
      testReduceAdd(4096, 4096, 4, 4, 4);

      // Blocks of 8 threads
      testReduceAdd(4096, 4096, 4, 2, 1);
      testReduceAdd(4096, 4096, 2, 4, 3);

      // Blocks of 4 threads
      testReduceAdd(4096, 4096, 2, 2, 1);
      testReduceAdd(4096, 4096, 2, 2, 5);

      // Blocks of 2 threads
      testReduceAdd(4096, 4096, 1, 2, 1);
      testReduceAdd(4096, 4096, 2, 1, 13);

      // Blocks of 1 thread
      testReduceAdd(4096, 4096, 1, 1, 1);
      testReduceAdd(4096, 4096, 1, 1, 17);

      // Typical grid and block size for processing 1280 x 720 images
      testReduceAdd(80, 45, 16, 16, 1);

      // 1280 x 720 image if each thread processes 4 pixels
      testReduceAdd(40, 28, 16, 16, 1);
   }

   private void testReduceAdd(int gridSizeX, int gridSizeY, int blockSizeX, int blockSizeY, int addValue) throws Exception
   {
      final long expectedResult = (long) addValue * gridSizeX * gridSizeY * blockSizeX * blockSizeY;

      CUstream_st stream = CUDAStreamManager.getStream();

      try (CUDAProgram program = new CUDAProgram("ReduceAddTest.cu", REDUCE_ADD_TEST_PROGRAM, UTILS_CU_HEADER, UTILS_CU_CONTENTS);
           CUDAKernel kernel = program.loadKernel("testReduceAdd");

           dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);
           dim3 blockSize = new dim3(blockSizeX, blockSizeY, 1);

           IntPointer resultPointerDevice = new IntPointer();
           IntPointer resultPointerHost = new IntPointer(1L))
      {
         CUDATools.mallocAsync(resultPointerDevice, 1, stream);

         kernel.withInt(addValue).withPointer(resultPointerDevice).run(stream, gridSize, blockSize, Integer.BYTES * blockSizeX * blockSizeY);
         CUDATools.checkCUDAError(cudaStreamSynchronize(stream));

         CUDATools.memcpyAsync(resultPointerHost, resultPointerDevice, 1, stream);
         cudaFreeAsync(resultPointerDevice, stream);
         cudaStreamSynchronize(stream);

         assertEquals(expectedResult, Integer.toUnsignedLong(resultPointerHost.get()));
      }

      CUDAStreamManager.releaseStream(stream);
   }
}
