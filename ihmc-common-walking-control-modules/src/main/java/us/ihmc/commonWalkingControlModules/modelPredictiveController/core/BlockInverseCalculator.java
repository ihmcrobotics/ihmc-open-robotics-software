package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import us.ihmc.convexOptimization.quadraticProgram.InverseMatrixCalculator;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.tools.functional.IntDoubleConsumer;

import java.util.function.IntUnaryOperator;

/**
 * This is a custom inverse matrix calculator that exploits the block diagonal nature of the cost Hessian in the MPC problem. Note that this has the underlying
 * assumption that the Hessian is block diagonal, which is not the case if the continuity functions are set as objectives, rather than constraints.
 */
public class BlockInverseCalculator implements InverseMatrixCalculator<NativeMatrix>
{
   private final LinearMPCIndexHandler indexHandler;

   private final NativeMatrix blockToInvert = new NativeMatrix(0, 0);
   private final NativeMatrix invertedBlock = new NativeMatrix(0, 0);
   private final IntUnaryOperator blockSizeProvider;

   public BlockInverseCalculator(LinearMPCIndexHandler indexHandler, IntUnaryOperator blockSizeProvider)
   {
      this.indexHandler = indexHandler;
      this.blockSizeProvider = blockSizeProvider;
   }

   @Override
   public void computeInverse(NativeMatrix matrix, NativeMatrix inverseMatrixToPack)
   {
      inverseMatrixToPack.reshape(indexHandler.getTotalProblemSize(), indexHandler.getTotalProblemSize());
      inverseMatrixToPack.zero();

      for (int i = 0; i < indexHandler.getNumberOfSegments(); i++)
      {
         int start = indexHandler.getComCoefficientStartIndex(i);
         int blockSize = blockSizeProvider.applyAsInt(i);
         int end = start + blockSize;

         blockToInvert.reshape(blockSize, blockSize);
         invertedBlock.reshape(blockSize, blockSize);
         blockToInvert.zero();
         invertedBlock.zero();

         blockToInvert.insert(matrix, start, end, start, end, 0, 0);

         invertedBlock.invert(blockToInvert);

         inverseMatrixToPack.insert(invertedBlock, start, start);
      }
   }
}
