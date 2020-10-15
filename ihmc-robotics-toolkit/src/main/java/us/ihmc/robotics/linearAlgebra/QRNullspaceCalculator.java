package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.QRDecomposition;

import us.ihmc.commons.MathTools;

public class QRNullspaceCalculator implements NullspaceCalculator
{
   private final QRDecomposition<DMatrixRMaj> decomposer;

   private final DMatrixRMaj nullspace;
   private final DMatrixRMaj Q;
   private final DMatrixRMaj R;

   private final DMatrixRMaj nullspaceProjector;
   private final DMatrixRMaj tempMatrixForProjectionInPlace;

   public QRNullspaceCalculator(int matrixSize)
   {
      MathTools.checkIntervalContains(matrixSize, 1, Integer.MAX_VALUE);

      nullspaceProjector = new DMatrixRMaj(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DMatrixRMaj(matrixSize, matrixSize);

      decomposer = DecompositionFactory_DDRM.qr(matrixSize, matrixSize);
      nullspace = new DMatrixRMaj(matrixSize, matrixSize);
      Q = new DMatrixRMaj(matrixSize, matrixSize);
      R = new DMatrixRMaj(matrixSize, matrixSize);
   }

   /**
    * Perform a projection in place as follows:
    * <p>
    * N = nullspace(B)
    * </p>
    * <p>
    * A = A * NN<sup>T</sup>)
    * </p>
    * @param matrixToProjectOntoNullspace the matrix to be projected, A in the equation. Modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection, B in the equation. Not Modified.
    */
   @Override
   public void projectOntoNullspace(DMatrixRMaj matrixToProjectOntoNullspace, DMatrixRMaj matrixToComputeNullspaceOf)
   {
      tempMatrixForProjectionInPlace.set(matrixToProjectOntoNullspace);
      projectOntoNullspace(tempMatrixForProjectionInPlace, matrixToComputeNullspaceOf, matrixToProjectOntoNullspace);
   }

   /**
    * Project {@code matrixToProjectOntoNullspace} onto the nullspace of {@code matrixToComputeNullspaceOf} as follows:
    * <p>
    * N = nullspace(B)
    * </p>
    * <p>
    * C = A * NN<sup>T</sup>)
    * </p>
    * @param matrixToProjectOntoNullspace the matrix to be projected, A in the equation. Not modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection, B in the equation. Not Modified.
    * @param projectedMatrixToPack matrix to store the resulting projection, C in the equation. Modified.
    */
   @Override
   public void projectOntoNullspace(DMatrixRMaj matrixToProjectOntoNullspace, DMatrixRMaj matrixToComputeNullspaceOf, DMatrixRMaj projectedMatrixToPack)
   {
      computeNullspaceProjector(matrixToComputeNullspaceOf, nullspaceProjector);
      CommonOps_DDRM.mult(matrixToProjectOntoNullspace, nullspaceProjector, projectedMatrixToPack);
   }

   /**
    * Compute the nullspace projector of the given matrix as follows:
    * <p>
    * &Nu; = NN<sup>T</sup>
    * </p>
    * Where N<sup>+</sup> is the nullspace of {@param matrixToComputeNullspaceOf}.
    * A svd decomposition solver is used to compute N.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection. Not Modified.
    * @param nullspaceProjectorToPack matrix to store the resulting nullspace matrix. Modified.
    */
   @Override
   public void computeNullspaceProjector(DMatrixRMaj matrixToComputeNullspaceOf, DMatrixRMaj nullspaceProjectorToPack)
   {
      int nullity = Math.max(matrixToComputeNullspaceOf.getNumCols() - matrixToComputeNullspaceOf.getNumRows(), 0);
      computeNullspace(nullspace, matrixToComputeNullspaceOf, nullity);

      nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      CommonOps_DDRM.multOuter(nullspace, nullspaceProjectorToPack);
   }

   private final DMatrixRMaj transposed = new DMatrixRMaj(0, 0);
   private void computeNullspace(DMatrixRMaj nullspaceToPack, DMatrixRMaj matrixToComputeNullspaceOf, int nullity)
   {
      int size = matrixToComputeNullspaceOf.getNumCols();
      int rank = matrixToComputeNullspaceOf.getNumRows();
      nullspaceToPack.reshape(size, nullity);
      Q.reshape(size, size);
      R.reshape(size, rank);
      transposed.reshape(size, rank);

      CommonOps_DDRM.transpose(matrixToComputeNullspaceOf, transposed);
      decomposer.decompose(transposed);
      decomposer.getQ(Q, false);

      CommonOps_DDRM.extract(Q, 0, Q.getNumRows(), Q.getNumCols() - nullity, Q.getNumCols(), nullspaceToPack, 0, 0);
   }
}
