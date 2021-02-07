package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.interfaces.linsol.LinearSolverSparse;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.CommonOps_DSCC;
import org.ejml.sparse.csc.factory.LinearSolverFactory_DSCC;
import us.ihmc.convexOptimization.quadraticProgram.InverseCostCalculator;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.matrixlib.NativeMatrix;

public class BlockInverseCalculator implements InverseCostCalculator<NativeMatrix>
{
   private final LinearMPCIndexHandler indexHandler;

   private final NativeMatrix blockToInvert = new NativeMatrix(0, 0);
   private final NativeMatrix invertedBlock = new NativeMatrix(0, 0);

   public BlockInverseCalculator(LinearMPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   @Override
   public void computeInverse(NativeMatrix matrix, NativeMatrix inverseMatrix)
   {
      inverseMatrix.reshape(indexHandler.getTotalProblemSize(), indexHandler.getTotalProblemSize());
      inverseMatrix.zero();

      for (int i = 0; i < indexHandler.getNumberOfSegments(); i++)
      {
         int start = indexHandler.getComCoefficientStartIndex(i);
         int blockSize = indexHandler.getRhoCoefficientsInSegment(i) + LinearMPCIndexHandler.comCoefficientsPerSegment;

         blockToInvert.reshape(blockSize, blockSize);
         invertedBlock.reshape(blockSize, blockSize);
         blockToInvert.zero();
         invertedBlock.zero();

         // TODO add a set block method
         blockToInvert.addBlock(matrix, 0, 0, start, start, blockSize, blockSize, 1.0);

         invertedBlock.invert(blockToInvert);

         // TODO add a set block method
         inverseMatrix.addBlock(invertedBlock, start, start, 0, 0, blockSize, blockSize, 1.0);
      }
   }
}
