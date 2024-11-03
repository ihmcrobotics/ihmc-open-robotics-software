package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.matrixlib.MatrixTools;

public class DampedSVDNullspaceCalculator implements DampedNullspaceCalculator
{
   private final SingularValueDecomposition_F64<DMatrixRMaj> decomposer;
   private final DMatrixRMaj sigma;
   private final DMatrixRMaj sigmaDampedSigma;
   private final DMatrixRMaj v;

   private final DMatrixRMaj nullspace;
   private final DMatrixRMaj Q;

   private final DMatrixRMaj nullspaceProjector;
   private final DMatrixRMaj tempMatrixForProjectionInPlace;

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);

   private double alpha = 0.0;

   public DampedSVDNullspaceCalculator(int matrixSize, double alpha)
   {
      MathTools.checkIntervalContains(matrixSize, 1, Integer.MAX_VALUE);

      this.alpha = alpha;

      nullspaceProjector = new DMatrixRMaj(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DMatrixRMaj(matrixSize, matrixSize);

      decomposer = DecompositionFactory_DDRM.svd(matrixSize, matrixSize, false, true, false);
      sigma = new DMatrixRMaj(matrixSize, matrixSize);
      sigmaDampedSigma = new DMatrixRMaj(matrixSize, matrixSize);
      v = new DMatrixRMaj(matrixSize, matrixSize);
      nullspace = new DMatrixRMaj(matrixSize, matrixSize);    // oversized, using reshape later
      Q = new DMatrixRMaj(matrixSize, matrixSize);    // oversized, using reshape later
   }

   @Override
   public void setPseudoInverseAlpha(double alpha)
   {
      this.alpha = alpha;
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

      computeNullspace(nullspace, Q, matrixToComputeNullspaceOf, nullity);

      nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      if (alpha == 0.0)
      {
         nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
         CommonOps_DDRM.multOuter(nullspace, nullspaceProjectorToPack);
      }
      else
      {
         computeDampedSigmaOperator(sigmaDampedSigma, sigma, alpha);
         tempMatrix.reshape(v.getNumRows(), sigmaDampedSigma.getNumCols());

         DiagonalMatrixTools.postMult(Q, sigmaDampedSigma, tempMatrix);
         CommonOps_DDRM.multTransB(-1.0, tempMatrix, Q, nullspaceProjectorToPack);
         MatrixTools.addDiagonal(nullspaceProjectorToPack, 1.0);
      }

   }

   private void computeNullspace(DMatrixRMaj nullspaceToPack, DMatrixRMaj QToPack, DMatrixRMaj matrixToComputeNullspaceOf, int nullity)
   {
      nullspaceToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), nullity);
      decomposer.decompose(matrixToComputeNullspaceOf);

      sigma.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumRows());
      decomposer.getW(sigma);

      v.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      QToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols() - nullity);
      boolean transposed = false;
      decomposer.getV(v, transposed);

      CommonOps_DDRM.extract(v, 0, v.getNumRows(), 0, v.getNumCols() - nullity, QToPack, 0, 0);
      CommonOps_DDRM.extract(v, 0, v.getNumRows(), v.getNumCols() - nullity, v.getNumCols(), nullspaceToPack, 0, 0);
   }

   private void computeDampedSigmaOperator(DMatrixRMaj dampedSigmaToPack, DMatrixRMaj sigma, double alpha)
   {
      int minor = Math.min(sigma.getNumRows(), sigma.getNumCols());
      dampedSigmaToPack.reshape(minor, minor);
      dampedSigmaToPack.zero();

      for (int i = 0; i < minor; i++)
      {
         double sigmaValue = sigma.get(i, i);
         double dampedSigma = sigmaValue * sigmaValue / (sigmaValue * sigmaValue + alpha * alpha);
         dampedSigmaToPack.set(i, i, dampedSigma);
      }
   }
}
