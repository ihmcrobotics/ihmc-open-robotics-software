package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;

import java.util.Arrays;

public class LQRTools
{
   public static void matrixExponential(DenseMatrix64F a, DenseMatrix64F b, int termsForExpansion)
   {
      DenseMatrix64F interiorGuy = CommonOps.identity(a.numRows);
      int denominator = 1;
      CommonOps.setIdentity(b);
      for (int k = 1; k < termsForExpansion; k++)
      {
         denominator *= k;
         CommonOps.mult(a, interiorGuy, interiorGuy);
         CommonOps.addEquals(b, 1.0 / denominator, interiorGuy);
      }
   }

   /**
    * P<sup>T</sup>X + P A - P B R<sup>-1</sup> B<sup>T</sup> P + Q = 0,
    * solves and returns P
    */
   public static DenseMatrix64F solveContinuousAlgebraicRiccatiEquation(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R)
   {
      DenseMatrix64F RInverse = new DenseMatrix64F(R);
      CommonOps.invert(RInverse);

      DenseMatrix64F BTranspose = new DenseMatrix64F(B);
      CommonOps.transpose(B);

      DenseMatrix64F ATranspose = new DenseMatrix64F(A);
      CommonOps.transpose(ATranspose);

      DenseMatrix64F BRInverseBTranspose = new DenseMatrix64F(B.numRows, B.numRows);
      NativeCommonOps.multQuad(BTranspose, RInverse, BRInverseBTranspose);

      int size = A.numRows;
      DenseMatrix64F Z = new DenseMatrix64F(2 * size, 2 * size);

      MatrixTools.setMatrixBlock(Z, 0, 0, A, 0, 0, size, size, 1.0);
      MatrixTools.setMatrixBlock(Z, 0, 3, BRInverseBTranspose, 0, 0, size, size, -1.0);
      MatrixTools.setMatrixBlock(Z, 3, 0, Q, 0, 0, size, size, -1.0);
      MatrixTools.setMatrixBlock(Z, 3, 3, ATranspose, 0, 0, size, size, -1.0);

      EigenDecomposition<DenseMatrix64F> decomposer = DecompositionFactory.eig(2 * size, false, false);
      decomposer.decompose(Z);
      int numberOfEigenValues = decomposer.getNumberOfEigenvalues();
      double[] eigenValues = new double[numberOfEigenValues];
      for (int i = 0; i < numberOfEigenValues; i++)
         eigenValues[i] = decomposer.getEigenvalue(i).getReal();
      Arrays.sort(eigenValues);

      DenseMatrix64F U2 = new DenseMatrix64F(size, size);
      DenseMatrix64F U1 = new DenseMatrix64F(size, size);

      for (int i = 0; i < size; i++)
      {
         U2.set(i, i, eigenValues[i]);
         U1.set(i, i, eigenValues[i + size]);
      }

      DenseMatrix64F U1Inverse = new DenseMatrix64F(U1);
      CommonOps.invert(U1Inverse);

      DenseMatrix64F P = new DenseMatrix64F(size, size);
      CommonOps.multAddTransA(U2, U1Inverse, P);

      return P;
   }
}
