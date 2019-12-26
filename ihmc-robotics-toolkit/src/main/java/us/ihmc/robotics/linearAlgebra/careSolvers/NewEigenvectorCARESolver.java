package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

public class NewEigenvectorCARESolver extends AbstractCARESolver
{
   private final EigenDecomposition<DenseMatrix64F> eigen = DecompositionFactory.eig(0, true);

   private final DenseMatrix64F EInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F EInverseTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F EInverseA = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F EInverseATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F EInverseMEInverseTranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F hamiltonian = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F u = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u1 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u2 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u1Inv = new DenseMatrix64F(0, 0);

   /** {@inheritDoc} */
   public DenseMatrix64F computeP()
   {
      EInverseATranspose.reshape(n, n);

      if (hasE)
      {
         EInverse.reshape(n, n);
         EInverseA.reshape(n, n);
         EInverseTranspose.reshape(n, n);
         NativeCommonOps.invert(E, EInverse);
         CommonOps.mult(E, A, EInverseA);

         CommonOps.transpose(EInverse, EInverseTranspose);
         NativeCommonOps.multQuad(EInverseTranspose, M, EInverseMEInverseTranspose);
      }
      else
      {
         EInverseA.set(A);
         EInverseMEInverseTranspose.set(M);
      }
      CommonOps.transpose(EInverseA, EInverseATranspose);

      // defining Hamiltonian
      hamiltonian.reshape(2 * n, 2 * n);
      MatrixTools.setMatrixBlock(hamiltonian, 0, 0, EInverseA, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonian, 0, n, EInverseMEInverseTranspose, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonian, n, 0, Q, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonian, n, n, EInverseATranspose, 0, 0, n, n, -1.0);

      // Eigen decomposition
      eigen.decompose(hamiltonian);
      u.reshape(2 * n, n);
      u1.reshape(n, n);
      u2.reshape(n, n);
      int valueIdx = 0;
      for (int i = 0; i < 2 * n; i++)
      {
         if (eigen.getEigenvalue(i).getImaginary() != 0.0)
            throw new IllegalArgumentException("The imaginary part of the eigenvalue must be equal to zero.");

         if (eigen.getEigenvalue(i).getReal() < 0.0)
         {
            MatrixTools.setMatrixBlock(u, 0, valueIdx++, eigen.getEigenVector(i), 0, 0, 2 * n, 1, 1.0);
         }

         if (valueIdx == n)
            break;
      }

      // solving linear system
      // P = -U_2 * U_1^-1
      MatrixTools.setMatrixBlock(u1, 0, 0, u, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(u2, 0, 0, u, n, 0, n, n, -1.0);

      u1Inv.reshape(n, n);
      NativeCommonOps.invert(u1, u1Inv);

      P.reshape(n, n);
      CommonOps.mult(u2, u1Inv, P);

      isUpToDate = true;

      return P;
   }
}
