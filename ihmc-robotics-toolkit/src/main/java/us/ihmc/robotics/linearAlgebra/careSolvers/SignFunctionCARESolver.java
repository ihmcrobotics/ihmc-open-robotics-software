package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.signFunction.NewtonSignFunction;
import us.ihmc.robotics.linearAlgebra.careSolvers.signFunction.SignFunction;

/**
 * <p>
 *    The solution is found by performing the Matrix Sign Function solution of the Hamiltonian, as outlined in
 *    that book
 * </p>
 */
public class SignFunctionCARESolver extends AbstractCARESolver
{
   private final DMatrixRMaj ETransposePE = new DMatrixRMaj(0, 0);

   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.qr(3, 3);
   private final SignFunction signFunction = new NewtonSignFunction();

   private final DMatrixRMaj EInverse = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj EInverseTranspose = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj EInverseA = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj EInverseATranspose = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj hamiltonian = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj identity = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj EInverseMEInverseTranspose = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj leftColumn = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj rightColumn = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj W = new DMatrixRMaj(0, 0);

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

      if (!signFunction.compute(hamiltonian))
         throw new RuntimeException("Error.");

      W.reshape(2 * n, 2 * n);
      signFunction.getW(W);

      identity.reshape(n, n);
      CommonOps_DDRM.setIdentity(identity);

      rightColumn.reshape(2 * n, n);
      leftColumn.reshape(2 * n, n);

      // M P = leftColumn
      MatrixTools.setMatrixBlock(rightColumn, 0, 0, W, 0, n, n, n, 1.0);
      MatrixTools.setMatrixBlock(rightColumn, n, 0, W, n, n, n, n, 1.0);
      MatrixTools.addMatrixBlock(rightColumn, n, 0, identity, 0, 0, n, n, 1.0);

      MatrixTools.setMatrixBlock(leftColumn, 0, 0, W, 0, 0, n, n, 1.0);
      MatrixTools.addMatrixBlock(leftColumn, 0, 0, identity, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(leftColumn, n, 0, W, n, 0, n, n, 1.0);

      ETransposePE.reshape(n, n);
      solver.setA(rightColumn);
      solver.solve(leftColumn, ETransposePE);

      if (!hasE)
      {
         P.set(ETransposePE);
      }
      else
      {
         NativeCommonOps.multQuad(EInverseTranspose, ETransposePE, P);
      }

      isUpToDate = true;

      return P;
   }
}
