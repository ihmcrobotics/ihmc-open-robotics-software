package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.interfaces.linsol.LinearSolverSparse;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.CommonOps_DSCC;
import org.ejml.sparse.csc.factory.LinearSolverFactory_DSCC;
import us.ihmc.convexOptimization.quadraticProgram.InverseCostCalculator;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.NativeCommonOps;

public class SparseInverseCalculator implements InverseCostCalculator<DMatrixSparseCSC>
{
   private static final boolean useSparse = true;
   private final LinearMPCIndexHandler indexHandler;

   private final LinearSolverSparse<DMatrixSparseCSC, DMatrixRMaj> solver = LinearSolverFactory_DSCC.cholesky(FillReducing.NONE);

   public SparseInverseCalculator(LinearMPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   private final DMatrixSparseCSC sparseBlockToInvert = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC sparseInvertedBlock = new DMatrixSparseCSC(0, 0);
   private final DMatrixRMaj denseBlockToInvert = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj denseInvertedBlock = new DMatrixRMaj(0, 0);
   private final DMatrixSparseCSC identity = new DMatrixSparseCSC(0, 0);

   @Override
   public void computeInverse(DMatrixSparseCSC matrix, DMatrixSparseCSC inverseMatrix)
   {
      inverseMatrix.reshape(indexHandler.getTotalProblemSize(), indexHandler.getTotalProblemSize());
      inverseMatrix.zero();

      if (useSparse)
      {
         for (int i = 0; i < indexHandler.getNumberOfSegments(); i++)
         {
            int start = indexHandler.getComCoefficientStartIndex(i);
            int blockSize = indexHandler.getRhoCoefficientsInSegment(i) + LinearMPCIndexHandler.comCoefficientsPerSegment;
            if (blockSize < 10)
               LogTools.info("help");

            sparseBlockToInvert.reshape(blockSize, blockSize);
            sparseInvertedBlock.reshape(blockSize, blockSize);
            sparseBlockToInvert.zero();
            sparseInvertedBlock.zero();

            fastExtractSparseBlock(matrix, start, start, blockSize, blockSize, sparseBlockToInvert);

            identity.reshape(blockSize, blockSize);
            CommonOps_DSCC.setIdentity(identity);

            solver.setA(sparseBlockToInvert);
            solver.solveSparse(identity, sparseInvertedBlock);

            fastSetSparseBlock(sparseInvertedBlock, inverseMatrix, start, start);
         }
      }
      else
      {
         for (int i = 0; i < indexHandler.getNumberOfSegments(); i++)
         {
            int start = indexHandler.getComCoefficientStartIndex(i);
            int blockSize = indexHandler.getRhoCoefficientsInSegment(i) + LinearMPCIndexHandler.comCoefficientsPerSegment;

            denseBlockToInvert.reshape(blockSize, blockSize);
            denseInvertedBlock.reshape(blockSize, blockSize);
            denseBlockToInvert.zero();
            denseInvertedBlock.zero();

            fastExtractDenseBlock(matrix, start, start, blockSize, blockSize, denseBlockToInvert);

            NativeCommonOps.invert(denseBlockToInvert, denseInvertedBlock);

            fastSetDenseBlock(denseInvertedBlock, inverseMatrix, start, start);
         }
      }
   }

   private static void fastExtractSparseBlock(DMatrixSparseCSC src, int startCol, int startRow, int numCols, int numRows, DMatrixSparseCSC dst)
   {
      int internalNz = src.col_idx[numCols + startCol] - src.col_idx[startCol];
      dst.growMaxLength(internalNz, false);

      int dstValIdx = 0;
      for (int col = 0; col < numCols; col++)
      {
         int colSrc = startCol + col;
         int idxS0 = src.col_idx[colSrc];
         int idxS1 = src.col_idx[colSrc + 1];

         dst.col_idx[col] = dstValIdx;
         for (int i = idxS0; i < idxS1; i++)
         {
            int row = src.nz_rows[i];
            if (row >= startRow && row < startRow + numRows)
            {
               dst.nz_rows[dstValIdx] = row - startRow;
               dst.nz_values[dstValIdx] = src.nz_values[i];
               dstValIdx++;
            }
         }
      }

      dst.col_idx[numCols] = dstValIdx;
      dst.nz_length = dstValIdx;
   }


   private static void fastSetSparseBlock(DMatrixSparseCSC src, DMatrixSparseCSC dst, int startCol, int startRow)
   {
      int numCols = src.getNumCols();
      int currentLength = dst.getNonZeroLength();
      dst.growMaxLength(currentLength + src.nz_length, true);

      int valueIdx = currentLength;
      for (int colSrc = 0; colSrc < numCols; colSrc++)
      {
         dst.col_idx[startCol + colSrc] = valueIdx;

         int idxS0 = src.col_idx[colSrc];
         int idxS1 = src.col_idx[colSrc + 1];

         for (int i = idxS0; i < idxS1; i++)
         {
            dst.nz_rows[valueIdx] = src.nz_rows[i] + startRow;
            dst.nz_values[valueIdx] = src.nz_values[i];
            valueIdx++;
         }

      }

      for (int col = startCol + numCols; col < dst.getNumCols() + 1; col++)
      {
         dst.col_idx[col] = valueIdx;
      }

      dst.nz_length = valueIdx;
   }



   static void fastExtractDenseBlock(DMatrixSparseCSC src, int startCol, int startRow, int numCols, int numRows, DMatrixRMaj dst)
   {
      for (int colSrc = startCol; colSrc < startCol + numCols; colSrc++)
      {
         int idxS0 = src.col_idx[colSrc];
         int idxS1 = src.col_idx[colSrc + 1];

         for (int i = idxS0; i < idxS1; i++)
         {
            int row = src.nz_rows[i];
            if (row >= startRow && row < startRow + numRows)
            {
               dst.set(row - startRow, colSrc - startCol, src.nz_values[i]);
            }
         }
      }
   }

   private static void fastSetDenseBlock(DMatrixRMaj src, DMatrixSparseCSC dst, int startCol, int startRow)
   {
      int numCols = src.getNumCols();
      int numRows = src.getNumRows();
      int currentLength = dst.getNonZeroLength();
      dst.growMaxLength(currentLength + numCols * numRows, true);

      int valueIdx = currentLength;
      for (int colSrc = 0; colSrc < numCols; colSrc++)
      {
         dst.col_idx[startCol + colSrc] = valueIdx;

         for (int rowSrc = 0; rowSrc < numRows; rowSrc++)
         {
            dst.nz_rows[valueIdx] = rowSrc + startRow;
            dst.nz_values[valueIdx] = src.unsafe_get(rowSrc, colSrc);

            valueIdx++;
         }

         dst.col_idx[startCol + colSrc + 1] = valueIdx;
      }

      for (int col = startCol + numCols; col < dst.getNumCols() + 1; col++)
      {
         dst.col_idx[col] = valueIdx;
      }

      dst.nz_length = valueIdx;
   }
}
