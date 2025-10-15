package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BoolPointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Implementation of Non-Maximum Suppression (NMS) using CUDA for parallel processing.
 * <p>
 * Given a set of bounding boxes, NMS groups the overlapping boxes and selects the best box in each group.
 * Since object detection algorithms tend to produce many potential bounding boxes
 * for each detected object, NMS is applied to find the most representative box of each object.
 * <p>
 * Intersection over Union (IoU) is used to measure overlap.
 * Essentially, if two boxes don't overlap their IoU = 0 and if they are identical their IoU = 1.
 * Lower threshold values will result in more boxes being removed, while higher values will remove fewer boxes.
 * <p>
 * For further reading, see: <a href="https://www.geeksforgeeks.org/what-is-non-maximum-suppression/">What is Non-Maximum Suppression</a>
 */
public class CUDANonMaximumSuppression implements AutoCloseable
{
   private static final int MAX_THREADS_PER_BLOCK = CUDATools.getMaxThreadsPerBlock();
   private static final int BLOCK_DIM_2D = 16;
   private static final int BLOCK_DIM_1D = 256;

   private final CUstream_st stream;
   private final CUDAProgram program;
   private final CUDAKernel checkInclusionKernel;
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

         checkInclusionKernel = program.loadKernel("checkInclusion");
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
   public int run(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices)
   {
      return runAsync(inputBoxes, boxCount, overlapThreshold, outputIncludedIndices, stream);
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
    * @param cudaStream            CUDA stream to run on.
    * @return Number of included indices.
    */
   public int runAsync(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices, CUstream_st cudaStream)
   {
      if (boxCount > MAX_THREADS_PER_BLOCK)
         return runSlowAsync(inputBoxes, boxCount, overlapThreshold, outputIncludedIndices, cudaStream);
      else
         return runFastAsync(inputBoxes, boxCount, overlapThreshold, outputIncludedIndices, cudaStream);
   }

   /**
    * Run a non-maximum suppression algorithm on boxes, using a fast reduction kernel.
    * <p>
    * The fast reduction kernel is limited on the number of boxes it can handle.
    * Typically, the maximum number of boxes is 512 or 1024 (depending on the GPU).
    * If you're unsure if there are too many boxes, use {@link #runAsync(FloatPointer, int, float, IntPointer, CUstream_st)}.
    *
    * @param inputBoxes            Boxes to run NMS on (x, y, width, height, score)
    * @param boxCount              Number of boxes in the input
    * @param overlapThreshold      Minimum Intersection over Union (IoU) value used for grouping boxes. Between 0.0 and 1.0.
    *                              The highest score box of each group will be kept, and the rest removed.
    * @param outputIncludedIndices Array of indices which have been included.
    *                              Should have memory allocated for at least {@code boxCount} elements.
    * @return Number of included indices.
    */
   public int runFast(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices)
   {
      int count = runFastAsync(inputBoxes, boxCount, overlapThreshold, outputIncludedIndices, stream);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      return count;
   }

   /**
    * Run a non-maximum suppression algorithm on boxes, using a fast reduction kernel.
    * <p>
    * The fast reduction kernel is limited on the number of boxes it can handle.
    * Typically, the maximum number of boxes is 512 or 1024 (depending on the GPU).
    * If you're unsure if there are too many boxes, use {@link #runAsync(FloatPointer, int, float, IntPointer, CUstream_st)}.
    *
    * @param inputBoxes            Boxes to run NMS on (x, y, width, height, score)
    * @param boxCount              Number of boxes in the input
    * @param overlapThreshold      Minimum Intersection over Union (IoU) value used for grouping boxes. Between 0.0 and 1.0.
    *                              The highest score box of each group will be kept, and the rest removed.
    * @param outputIncludedIndices Array of indices which have been included.
    *                              Should have memory allocated for at least {@code boxCount} elements.
    * @param cudaStream            CUDA stream to run on.
    * @return Number of included indices.
    */
   public int runFastAsync(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices, CUstream_st cudaStream)
   {
      try (BoolPointer inclusionMatrix = new BoolPointer())
      {
         CUDATools.mallocAsync(inclusionMatrix, (long) boxCount * boxCount, cudaStream);

         createInclusionMatrix(inputBoxes, boxCount, overlapThreshold, inclusionMatrix, cudaStream);

         int count = runFastReduction(inclusionMatrix, boxCount, outputIncludedIndices, cudaStream);

         CUDATools.checkCUDAError(cudaFreeAsync(inclusionMatrix, cudaStream));

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
   public int runSlow(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices)
   {
      int count = runSlowAsync(inputBoxes, boxCount, overlapThreshold, outputIncludedIndices, stream);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      return count;
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
    * @param cudaStream            CUDA stream to run on.
    * @return Number of included indices.
    */
   public int runSlowAsync(FloatPointer inputBoxes, int boxCount, float overlapThreshold, IntPointer outputIncludedIndices, CUstream_st cudaStream)
   {
      try (BoolPointer inclusionMatrix = new BoolPointer())
      {
         CUDATools.mallocAsync(inclusionMatrix, (long) boxCount * boxCount, cudaStream);

         createInclusionMatrix(inputBoxes, boxCount, overlapThreshold, inclusionMatrix, cudaStream);

         int count = runSlowReduction(inclusionMatrix, boxCount, outputIncludedIndices, cudaStream);

         CUDATools.checkCUDAError(cudaFreeAsync(inclusionMatrix, cudaStream));

         return count;
      }
   }

   private void createInclusionMatrix(FloatPointer inputBoxes, int boxCount, float overlapThreshold, BoolPointer outputInclusionMatrix, CUstream_st stream)
   {
      int divisor = 4 * BLOCK_DIM_2D;
      int gridSize2D = (boxCount + divisor - 1) / divisor;

      try (dim3 mappingBlockSize = new dim3(BLOCK_DIM_2D, BLOCK_DIM_2D, 1);
           dim3 mappingGridSize = new dim3(gridSize2D, gridSize2D, 1);
           FloatPointer boxes = new FloatPointer();)
      {
         CUDATools.mallocAsync(boxes, 5L * boxCount, stream);
         CUDATools.memcpyAsync(boxes, inputBoxes, 5L * boxCount, stream);

         checkInclusionKernel.withPointer(boxes)
                             .withInt(boxCount)
                             .withFloat(overlapThreshold)
                             .withPointer(outputInclusionMatrix)
                             .run(stream, mappingGridSize, mappingBlockSize, 0);

         CUDATools.checkCUDAError(cudaFreeAsync(boxes, stream));
      }
   }

   private int runFastReduction(BoolPointer inclusionMatrix, int boxCount, IntPointer outputIncludedIndices, CUstream_st stream)
   {
      try (dim3 reduceBlockSize = new dim3(boxCount, 1, 1);
           dim3 reduceGridSize = new dim3(boxCount, 1, 1);
           IntPointer includedIndices = new IntPointer();
           IntPointer includedCount = new IntPointer())
      {
         CUDATools.checkCUDAError(cudaMallocHost(includedCount, 1));
         includedCount.put(0);

         CUDATools.mallocAsync(includedIndices, boxCount, stream);

         fastReductionKernel.withPointer(inclusionMatrix)
                            .withInt(boxCount)
                            .withPointer(includedIndices)
                            .withPointer(includedCount)
                            .run(stream, reduceGridSize, reduceBlockSize, 0);

         CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
         int count = includedCount.get();
         CUDATools.checkCUDAError(cudaFreeHost(includedCount));

         CUDATools.memcpyAsync(outputIncludedIndices, includedIndices, count, stream);
         CUDATools.checkCUDAError(cudaFreeAsync(includedIndices, stream));

         return count;
      }
   }

   private int runSlowReduction(BoolPointer inclusionMatrix, int boxCount, IntPointer outputIncludedIndices, CUstream_st stream)
   {
      int gridSize1D = (boxCount + BLOCK_DIM_1D - 1) / BLOCK_DIM_1D;

      try (dim3 reduceBlockSize = new dim3(BLOCK_DIM_1D, 1, 1);
           dim3 reduceGridSize = new dim3(gridSize1D, 1, 1);
           IntPointer includedIndices = new IntPointer();
           IntPointer includedCount = new IntPointer())
      {
         CUDATools.checkCUDAError(cudaMallocHost(includedCount, 1));
         includedCount.put(0);

         CUDATools.mallocAsync(includedIndices, boxCount, stream);

         slowReductionKernel.withPointer(inclusionMatrix)
                            .withInt(boxCount)
                            .withPointer(includedIndices)
                            .withPointer(includedCount)
                            .run(stream, reduceGridSize, reduceBlockSize, 0);

         CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
         int count = includedCount.get();
         CUDATools.checkCUDAError(cudaFreeHost(includedCount));

         CUDATools.memcpyAsync(outputIncludedIndices, includedIndices, count, stream);
         CUDATools.checkCUDAError(cudaFreeAsync(includedIndices, stream));

         return count;
      }
   }

   @Override
   public void close()
   {
      program.close();
      checkInclusionKernel.close();
      fastReductionKernel.close();
      slowReductionKernel.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
