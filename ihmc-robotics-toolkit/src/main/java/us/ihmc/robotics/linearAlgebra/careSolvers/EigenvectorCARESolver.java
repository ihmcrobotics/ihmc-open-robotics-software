package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;

import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

/**
 * This solver computes the solution to the algebraic Riccati equation using the Eigen vectors of the Hamiltonian of the problem. See that you can form the
 * Hamiltonian from the cost function and its adjoints as part of the optimal control problem. You can then deconstruct this into Eigen values and Eigen vectors
 * using a standard Eigen value decomposition. Because you are deconstructing a Hamiltonian, half of the eigen values are stable, and half are unstable.
 * You can then find the solution to the algebraic Riccati equation using the top and bottom blocks of the stable eigen vectors.
 *
 * <p>
 *    This approach is outlined in https://en.wikipedia.org/wiki/Algebraic_Riccati_equation.
 * </p>
 * <p>
 *    NOTE: This solver is fast, but requires that the Hamiltonian has only REAL eigen values, so that the eigen vector decomposition is possible. In general,
 *    this is not a valid assumption. If you know for your system that it is, then this returns a fast solution.
 * </p>
 */
public class EigenvectorCARESolver extends AbstractCARESolver
{
   private final EigenDecomposition_F64<DMatrixRMaj> eigen = DecompositionFactory_DDRM.eig(0, true);

   private final DMatrixRMaj EInverse = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj EInverseTranspose = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj EInverseA = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj EInverseATranspose = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj EInverseMEInverseTranspose = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj hamiltonian = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj u = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj u1 = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj u2 = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj u1Inv = new DMatrixRMaj(0, 0);

   /** {@inheritDoc} */
   public DMatrixRMaj computeP()
   {
      EInverseATranspose.reshape(n, n);

      if (hasE)
      {
         EInverse.reshape(n, n);
         EInverseA.reshape(n, n);
         EInverseTranspose.reshape(n, n);
         NativeCommonOps.invert(E, EInverse);
         CommonOps_DDRM.mult(E, A, EInverseA);

         CommonOps_DDRM.transpose(EInverse, EInverseTranspose);
         NativeCommonOps.multQuad(EInverseTranspose, M, EInverseMEInverseTranspose);
      }
      else
      {
         EInverseA.set(A);
         EInverseMEInverseTranspose.set(M);
      }
      CommonOps_DDRM.transpose(EInverseA, EInverseATranspose);

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
      CommonOps_DDRM.mult(u2, u1Inv, P);

      isUpToDate = true;

      return P;
   }
}
