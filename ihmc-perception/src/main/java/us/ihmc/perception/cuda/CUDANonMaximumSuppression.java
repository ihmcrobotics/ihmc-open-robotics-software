package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BoolPointer;
import org.bytedeco.javacpp.FloatPointer;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class CUDANonMaximumSuppression implements AutoCloseable
{
   private static final int BLOCK_DIM_2D = 16;
   private static final int BLOCK_DIM_1D = 256;

   private final CUstream_st stream;
   private final CUDAProgram program;
   private final CUDAKernel mappingKernel;
   private final CUDAKernel fastReductionKernel;
   private final CUDAKernel slowReductionKernel;

   public CUDANonMaximumSuppression() throws Exception
   {
      stream = CUDAStreamManager.getStream();

      URL programURL = CUDADepthColorizer.class.getResource("NonMaximumSuppression.cu");
      URL utilsURL = CUDATools.getUtilsFile();
      program = new CUDAProgram(programURL, utilsURL);

      mappingKernel = program.loadKernel("checkInclusion");
      fastReductionKernel = program.loadKernel("reduceFast");
      slowReductionKernel = program.loadKernel("reduceSlow");
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
    * @param outputInclusionVector Array of boolean values indicating whether a box is kept or removed.
    */
   public void run(FloatPointer inputBoxes, long boxCount, float overlapThreshold, BoolPointer outputInclusionVector)
   {
      if (boxCount > CUDATools.maxThreadsPerBlock())
         runSlow(inputBoxes, boxCount, overlapThreshold, outputInclusionVector);
      else
         runFast(inputBoxes, boxCount, overlapThreshold, outputInclusionVector);
   }

   /**
    * Run a non-maximum suppression algorithm on boxes, using a fast reduction kernel.
    * <p>
    * The fast reduction kernel is limited on the number of boxes it can handle.
    * Typically, the maximum number of boxes is 512 or 1024 (depending on the GPU).
    * If you're unsure if there are too many boxes, use {@link #run(FloatPointer, long, float, BoolPointer)}.
    *
    * @param inputBoxes            Boxes to run NMS on (x, y, width, height, score)
    * @param boxCount              Number of boxes in the input
    * @param overlapThreshold      Minimum Intersection over Union (IoU) value used for grouping boxes. Between 0.0 and 1.0.
    *                              The highest score box of each group will be kept, and the rest removed.
    * @param outputInclusionVector Array of boolean values indicating whether a box is kept or removed.
    */
   public void runFast(FloatPointer inputBoxes, long boxCount, float overlapThreshold, BoolPointer outputInclusionVector)
   {
      int divisor = 4 * BLOCK_DIM_2D;
      int gridSize2D = (int) (boxCount + divisor - 1) / divisor;

      try (dim3 mappingBlockSize = new dim3(BLOCK_DIM_2D, BLOCK_DIM_2D, 1);
           dim3 mappingGridSize = new dim3(gridSize2D, gridSize2D, 1);
           dim3 reduceBlockSize = new dim3((int) boxCount, 1, 1);
           dim3 reduceGridSize = new dim3((int) boxCount, 1, 1);
           FloatPointer boxes = new FloatPointer();
           BoolPointer inclusionMatrix = new BoolPointer();
           BoolPointer inclusionVector = new BoolPointer())
      {
         CUDATools.mallocAsync(boxes, 5 * boxCount, stream);
         CUDATools.memcpyAsync(boxes, inputBoxes, 5 * boxCount, stream);

         CUDATools.mallocAsync(inclusionMatrix, boxCount * boxCount, stream);
         CUDATools.mallocAsync(inclusionVector, boxCount, stream);

         mappingKernel.withPointer(boxes)
                      .withLong(boxCount)
                      .withFloat(overlapThreshold)
                      .withPointer(inclusionMatrix)
                      .run(stream, mappingGridSize, mappingBlockSize, 0);

         fastReductionKernel.withPointer(inclusionMatrix)
                            .withLong(boxCount)
                            .withPointer(inclusionVector)
                            .run(stream, reduceGridSize, reduceBlockSize, 0);

         CUDATools.memcpyAsync(outputInclusionVector, inclusionVector, boxCount, stream);
         cudaStreamSynchronize(stream);

         cudaFreeAsync(boxes, stream);
         cudaFreeAsync(inclusionMatrix, stream);
         cudaFreeAsync(inclusionVector, stream);
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
    * @param outputInclusionVector Array of boolean values indicating whether a box is kept or removed.
    */
   public void runSlow(FloatPointer inputBoxes, long boxCount, float overlapThreshold, BoolPointer outputInclusionVector)
   {
      int divisor = 4 * BLOCK_DIM_2D;
      int gridSize2D = (int) (boxCount + divisor - 1) / divisor;

      int gridSize1D = (int) (boxCount + BLOCK_DIM_1D - 1) / BLOCK_DIM_1D;

      try (dim3 mappingBlockSize = new dim3(BLOCK_DIM_2D, BLOCK_DIM_2D, 1);
           dim3 mappingGridSize = new dim3(gridSize2D, gridSize2D, 1);
           dim3 reduceBlockSize = new dim3(BLOCK_DIM_1D, 1, 1);
           dim3 reduceGridSize = new dim3(gridSize1D, 1, 1);
           FloatPointer boxes = new FloatPointer();
           BoolPointer inclusionMatrix = new BoolPointer();
           BoolPointer inclusionVector = new BoolPointer())
      {
         CUDATools.mallocAsync(boxes, 5 * boxCount, stream);
         CUDATools.memcpyAsync(boxes, inputBoxes, 5 * boxCount, stream);

         CUDATools.mallocAsync(inclusionMatrix, boxCount * boxCount, stream);
         CUDATools.mallocAsync(inclusionVector, boxCount, stream);

         mappingKernel.withPointer(boxes)
                      .withLong(boxCount)
                      .withFloat(overlapThreshold)
                      .withPointer(inclusionMatrix)
                      .run(stream, mappingGridSize, mappingBlockSize, 0);

         slowReductionKernel.withPointer(inclusionMatrix)
                            .withLong(boxCount)
                            .withPointer(inclusionVector)
                            .run(stream, reduceGridSize, reduceBlockSize, 0);

         CUDATools.memcpyAsync(outputInclusionVector, inclusionVector, boxCount, stream);
         cudaStreamSynchronize(stream);

         cudaFreeAsync(boxes, stream);
         cudaFreeAsync(inclusionMatrix, stream);
         cudaFreeAsync(inclusionVector, stream);
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
