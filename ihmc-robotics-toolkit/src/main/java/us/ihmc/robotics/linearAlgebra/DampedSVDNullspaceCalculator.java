package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;

public class DampedSVDNullspaceCalculator implements DampedNullspaceCalculator
{
   private final SingularValueDecomposition<DenseMatrix64F> decomposer;
   private final DenseMatrix64F sigma;
   private final DenseMatrix64F sigmaDampedSigma;
   private final DenseMatrix64F v;

   private final DenseMatrix64F nullspace;
   private final DenseMatrix64F Q;

   private final DenseMatrix64F nullspaceProjector;
   private final DenseMatrix64F tempMatrixForProjectionInPlace;

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   private double alpha = 0.0;

   public DampedSVDNullspaceCalculator(int matrixSize, double alpha)
   {
      MathTools.checkIntervalContains(matrixSize, 1, Integer.MAX_VALUE);

      this.alpha = alpha;

      nullspaceProjector = new DenseMatrix64F(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DenseMatrix64F(matrixSize, matrixSize);

      decomposer = DecompositionFactory.svd(matrixSize, matrixSize, false, true, false);
      sigma = new DenseMatrix64F(matrixSize, matrixSize);
      sigmaDampedSigma = new DenseMatrix64F(matrixSize, matrixSize);
      v = new DenseMatrix64F(matrixSize, matrixSize);
      nullspace = new DenseMatrix64F(matrixSize, matrixSize);    // oversized, using reshape later
      Q = new DenseMatrix64F(matrixSize, matrixSize);    // oversized, using reshape later
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
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf)
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
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F projectedMatrixToPack)
   {
      computeNullspaceProjector(matrixToComputeNullspaceOf, nullspaceProjector);
      CommonOps.mult(matrixToProjectOntoNullspace, nullspaceProjector, projectedMatrixToPack);
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
   public void computeNullspaceProjector(DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F nullspaceProjectorToPack)
   {
      int nullity = Math.max(matrixToComputeNullspaceOf.getNumCols() - matrixToComputeNullspaceOf.getNumRows(), 0);

      computeNullspace(nullspace, Q, matrixToComputeNullspaceOf, nullity);

      nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      if (alpha == 0.0)
      {
         nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
         CommonOps.multOuter(nullspace, nullspaceProjectorToPack);
      }
      else
      {
         computeDampedSigmaOperator(sigmaDampedSigma, sigma, alpha);
         tempMatrix.reshape(v.getNumRows(), sigmaDampedSigma.getNumCols());

         DiagonalMatrixTools.postMult(Q, sigmaDampedSigma, tempMatrix);
         CommonOps.multTransB(-1.0, tempMatrix, Q, nullspaceProjectorToPack);
         MatrixTools.addDiagonal(nullspaceProjectorToPack, 1.0);
      }

   }

   private void computeNullspace(DenseMatrix64F nullspaceToPack, DenseMatrix64F QToPack, DenseMatrix64F matrixToComputeNullspaceOf, int nullity)
   {
      nullspaceToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), nullity);
      decomposer.decompose(matrixToComputeNullspaceOf);

      sigma.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumRows());
      decomposer.getW(sigma);

      v.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      QToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols() - nullity);
      boolean transposed = false;
      decomposer.getV(v, transposed);

      CommonOps.extract(v, 0, v.getNumRows(), 0, v.getNumCols() - nullity, QToPack, 0, 0);
      CommonOps.extract(v, 0, v.getNumRows(), v.getNumCols() - nullity, v.getNumCols(), nullspaceToPack, 0, 0);
   }

   private void computeDampedSigmaOperator(DenseMatrix64F dampedSigmaToPack, DenseMatrix64F sigma, double alpha)
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
