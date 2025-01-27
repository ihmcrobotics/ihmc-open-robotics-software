package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BoolPointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class CUDANonMaximumSuppression implements AutoCloseable
{
   private static final int BLOCK_DIM_2D = 16;
   private static final int BLOCK_DIM_1D = 256;

   private final CUstream_st stream;
   private final CUDAProgram program;
   private final CUDAKernel mappingKernel;
   private final CUDAKernel fastReductionKernel;
   private final CUDAKernel slowReductionKernel;

   public CUDANonMaximumSuppression()
   {
      stream = CUDAStreamManager.getStream();

      URL programURL = CUDADepthColorizer.class.getResource("NonMaximumSuppression.cu");
      URL utilsURL = CUDATools.getUtilsFile();
      try
      {
         program = new CUDAProgram(programURL, utilsURL);

         mappingKernel = program.loadKernel("checkInclusion");
         fastReductionKernel = program.loadKernel("reduceFast");
         slowReductionKernel = program.loadKernel("reduceSlow");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Run a non-maximum suppression algorithm on boxes.
    * <p>
    * This method uses the fast reduction kernel if possible,
    * and defaults to the slow kernel otherwise.
    *
    * @param inputBoxes            Boxes to run NMS on (x, y, width, height, score)
    * @param boxCount              Number of boxes in the input
    * @param overlapThreshold      Minimum Intersection over Union (IoU) value used for grouping boxes. Between 0.0 and 1.0.
    *                              The highest score box of each group will be kept, and the rest removed.
    * @param outputIncludedIndices Array of indices which have been included.
    *                              Should have memory allocated for at least {@code boxCount} elements.
    * @return Number of included indices.
    */
   public long run(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices)
   {
      if (boxCount > CUDATools.maxThreadsPerBlock())
         return runSlow(inputBoxes, boxCount, overlapThreshold, outputIncludedIndices);
      else
         return runFast(inputBoxes, boxCount, overlapThreshold, outputIncludedIndices);
   }

   /**
    * Run a non-maximum suppression algorithm on boxes, using a fast reduction kernel.
    * <p>
    * The fast reduction kernel is limited on the number of boxes it can handle.
    * Typically, the maximum number of boxes is 512 or 1024 (depending on the GPU).
    * If you're unsure if there are too many boxes, use {@link #run(FloatPointer, int, float, IntPointer)}.
    *
    * @param inputBoxes            Boxes to run NMS on (x, y, width, height, score)
    * @param boxCount              Number of boxes in the input
    * @param overlapThreshold      Minimum Intersection over Union (IoU) value used for grouping boxes. Between 0.0 and 1.0.
    *                              The highest score box of each group will be kept, and the rest removed.
    * @param outputIncludedIndices Array of indices which have been included.
    *                              Should have memory allocated for at least {@code boxCount} elements.
    * @return Number of included indices.
    */
   public long runFast(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices)
   {
      int divisor = 4 * BLOCK_DIM_2D;
      int gridSize2D = (boxCount + divisor - 1) / divisor;

      try (dim3 mappingBlockSize = new dim3(BLOCK_DIM_2D, BLOCK_DIM_2D, 1);
           dim3 mappingGridSize = new dim3(gridSize2D, gridSize2D, 1);
           dim3 reduceBlockSize = new dim3(boxCount, 1, 1);
           dim3 reduceGridSize = new dim3(boxCount, 1, 1);
           FloatPointer boxes = new FloatPointer();
           BoolPointer inclusionMatrix = new BoolPointer();
           IntPointer includedIndices = new IntPointer();
           IntPointer includedCount = new IntPointer())
      {
         CUDATools.checkCUDAError(cudaMallocHost(includedCount, includedCount.sizeof()));

         CUDATools.mallocAsync(boxes, 5L * boxCount, stream);
         CUDATools.memcpyAsync(boxes, inputBoxes, 5L * boxCount, stream);

         CUDATools.mallocAsync(inclusionMatrix, (long) boxCount * boxCount, stream);
         CUDATools.mallocAsync(includedIndices, boxCount, stream);

         mappingKernel.withPointer(boxes)
                      .withInt(boxCount)
                      .withFloat(overlapThreshold)
                      .withPointer(inclusionMatrix)
                      .run(stream, mappingGridSize, mappingBlockSize, 0);

         fastReductionKernel.withPointer(inclusionMatrix)
                            .withInt(boxCount)
                            .withPointer(includedIndices)
                            .withPointer(includedCount)
                            .run(stream, reduceGridSize, reduceBlockSize, 0);

         CUDATools.memcpyAsync(outputIncludedIndices, includedIndices, boxCount, stream);
         cudaStreamSynchronize(stream);
         int count = includedCount.get();

         cudaFreeHost(includedCount);
         cudaFreeAsync(boxes, stream);
         cudaFreeAsync(inclusionMatrix, stream);
         cudaFreeAsync(includedIndices, stream);

         return count;
      }
   }

   /**
    * Run a non-maximum suppression algorithm on boxes, using a slow reduction kernel.
    * <p>
    * Use this method if the number of boxes is always greater than what the fast reduction kernel can handle.
    *
    * @param inputBoxes            Boxes to run NMS on (x, y, width, height, score)
    * @param boxCount              Number of boxes in the input
    * @param overlapThreshold      Minimum Intersection over Union (IoU) value used for grouping boxes. Between 0.0 and 1.0.
    *                              The highest score box of each group will be kept, and the rest removed.
    * @param outputIncludedIndices Array of indices which have been included.
    *                              Should have memory allocated for at least {@code boxCount} elements.
    * @return Number of included indices.
    */
   public long runSlow(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices)
   {
      int divisor = 4 * BLOCK_DIM_2D;
      int gridSize2D = (boxCount + divisor - 1) / divisor;

      int gridSize1D = (boxCount + BLOCK_DIM_1D - 1) / BLOCK_DIM_1D;

      try (dim3 mappingBlockSize = new dim3(BLOCK_DIM_2D, BLOCK_DIM_2D, 1);
           dim3 mappingGridSize = new dim3(gridSize2D, gridSize2D, 1);
           dim3 reduceBlockSize = new dim3(BLOCK_DIM_1D, 1, 1);
           dim3 reduceGridSize = new dim3(gridSize1D, 1, 1);
           FloatPointer boxes = new FloatPointer();
           BoolPointer inclusionMatrix = new BoolPointer();
           IntPointer includedIndices = new IntPointer();
           IntPointer includedCount = new IntPointer())
      {
         CUDATools.checkCUDAError(cudaMallocHost(includedCount, includedCount.sizeof()));

         CUDATools.mallocAsync(boxes, 5L * boxCount, stream);
         CUDATools.memcpyAsync(boxes, inputBoxes, 5L * boxCount, stream);

         CUDATools.mallocAsync(inclusionMatrix, (long) boxCount * boxCount, stream);
         CUDATools.mallocAsync(includedIndices, boxCount, stream);

         mappingKernel.withPointer(boxes)
                      .withInt(boxCount)
                      .withFloat(overlapThreshold)
                      .withPointer(inclusionMatrix)
                      .run(stream, mappingGridSize, mappingBlockSize, 0);

         slowReductionKernel.withPointer(inclusionMatrix)
                            .withInt(boxCount)
                            .withPointer(includedIndices)
                            .withPointer(includedCount)
                            .run(stream, reduceGridSize, reduceBlockSize, 0);

         CUDATools.memcpyAsync(outputIncludedIndices, includedIndices, boxCount, stream);
         cudaStreamSynchronize(stream);
         int count = includedCount.get();

         cudaFreeHost(includedCount);
         cudaFreeAsync(boxes, stream);
         cudaFreeAsync(inclusionMatrix, stream);
         cudaFreeAsync(includedIndices, stream);

         return count;
      }
   }

   @Override
   public void close()
   {
      program.close();
      mappingKernel.close();
      fastReductionKernel.close();
      slowReductionKernel.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
