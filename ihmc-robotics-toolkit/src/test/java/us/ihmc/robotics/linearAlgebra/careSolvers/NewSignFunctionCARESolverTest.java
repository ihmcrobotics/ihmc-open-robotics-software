package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.schur.SchurDecomposition;
import us.ihmc.robotics.linearAlgebra.careSolvers.schur.SchurDecompositionFactory;
import us.ihmc.robotics.linearAlgebra.careSolvers.signFunction.NewtonSignFunction;
import us.ihmc.robotics.linearAlgebra.careSolvers.signFunction.SignFunction;

public class NewSignFunctionCARESolverTest extends NewCARESolverTest
{
   @Override
   protected NewCARESolver getSolver()
   {
      return new NewSignFunctionCARESolver();
   }

   @Override
   protected double getEpsilon()
   {
      return 1e-4;
   }


   @Test
   public void testMatrixSignConstruction()
   {
      int n = 2;
      int m = 1;
      DenseMatrix64F A = new DenseMatrix64F(n, n);
      DenseMatrix64F B = new DenseMatrix64F(n, m);
      DenseMatrix64F C = new DenseMatrix64F(1, 2);
      DenseMatrix64F Q = new DenseMatrix64F(2, 2);
      DenseMatrix64F R = new DenseMatrix64F(1, 1);
      A.set(0, 0, -3);
      A.set(0, 1, 2);
      A.set(1, 0, 1);
      A.set(1, 1, 1);
      B.set(1, 0, 1);
      C.set(0, 0, 1);
      C.set(0, 1, -1);
      R.set(0, 0, 3);

      CommonOps.multInner(C, Q);

      NewCARESolver solver = getSolver();
      solver.setMatrices(A, B, null, Q, R);
      solver.computeP();

      DenseMatrix64F PExpected = new DenseMatrix64F(2, 2);
      PExpected.set(0, 0, 0.5895);
      PExpected.set(0, 1, 1.8216);
      PExpected.set(1, 0, 1.8216);
      PExpected.set(1, 1, 8.8188);

      DenseMatrix64F P = solver.getP();

      DenseMatrix64F S = new DenseMatrix64F(2, 2);
      DenseMatrix64F BTranspose = new DenseMatrix64F(B);
      DenseMatrix64F H = new DenseMatrix64F(4, 4);
      CommonOps.transpose(BTranspose);
      CARETools.computeM(BTranspose, R, null, S);
      CARETools.assembleHamiltonian(A, null, Q, S, H);

      SignFunction matrixSignFunction = new NewtonSignFunction();
      SchurDecomposition<DenseMatrix64F> schur = SchurDecompositionFactory.qrBased(4);
      matrixSignFunction.compute(H);
      schur.decompose(H);
      DenseMatrix64F T = schur.getT(null);
      DenseMatrix64F U = schur.getU(null);
      DenseMatrix64F Uinv = new DenseMatrix64F(U);
      NativeCommonOps.invert(U, Uinv);

      DenseMatrix64F Hexpected = new DenseMatrix64F(4, 4);
      DenseMatrix64F UTranspose = new DenseMatrix64F(U);
      CommonOps.transpose(UTranspose);
      NativeCommonOps.multQuad(UTranspose, T, Hexpected);

      EjmlUnitTests.assertEquals(Hexpected, H, 1e-5);

      DenseMatrix64F W = matrixSignFunction.getW(null);

      DenseMatrix64F I = new DenseMatrix64F(n, n);
      CommonOps.setIdentity(I);

      DenseMatrix64F M = new DenseMatrix64F(2 * n, n);
      DenseMatrix64F N = new DenseMatrix64F(2 * n, n);

      MatrixTools.setMatrixBlock(M, 0, 0, W, 0, n, n, n, 1.0);
      MatrixTools.setMatrixBlock(M, n, 0, W, n, n, n, n, 1.0);
      MatrixTools.addMatrixBlock(M, n, 0, I, 0, 0, n, n, 1.0);

      MatrixTools.setMatrixBlock(N, 0, 0, W, 0, 0, n, n, 1.0);
      MatrixTools.addMatrixBlock(N, 0, 0, I, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(N, n, 0, W, n, 0, n, n, 1.0);


      //      assertIsSymmetric(P);
      assertSolutionIsValid(A, B, Q, R, P, getEpsilon());
      EjmlUnitTests.assertEquals(PExpected, P, 1e-4);
   }

}
