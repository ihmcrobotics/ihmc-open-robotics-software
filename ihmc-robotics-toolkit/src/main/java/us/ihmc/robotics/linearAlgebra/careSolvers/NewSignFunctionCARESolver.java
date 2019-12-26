package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
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
public class NewSignFunctionCARESolver extends AbstractCARESolver
{
   private final DenseMatrix64F ETransposePE = new DenseMatrix64F(0, 0);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.qr(3, 3);
   private final SignFunction signFunction = new NewtonSignFunction();

   private final DenseMatrix64F EInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F EInverseTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F EInverseA = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F EInverseATranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F hamiltonian = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F identity = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F EInverseMEInverseTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F leftColumn = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F rightColumn = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F W = new DenseMatrix64F(0, 0);

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

      if (!signFunction.compute(hamiltonian))
         throw new RuntimeException("Error.");

      W.reshape(2 * n, 2 * n);
      signFunction.getW(W);

      identity.reshape(n, n);
      CommonOps.setIdentity(identity);

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
