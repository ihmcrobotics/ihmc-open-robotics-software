package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import sun.security.jgss.GSSHeader;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.Random;

public abstract class CARESolverTest
{
   private static final double epsilon = 1e-5;

   protected abstract CARESolver getSolver();

   @Test
   public void testSimple()
   {
      DenseMatrix64F A = CommonOps.identity(2);
      DenseMatrix64F B = CommonOps.identity(2);
      DenseMatrix64F Q = CommonOps.identity(2);
      DenseMatrix64F R = CommonOps.identity(2);

      DenseMatrix64F AInput = new DenseMatrix64F(A);
      DenseMatrix64F BInput = new DenseMatrix64F(B);
      DenseMatrix64F QInput = new DenseMatrix64F(Q);
      DenseMatrix64F RInput = new DenseMatrix64F(R);

      CARESolver solver = getSolver();
      solver.setMatrices(A, B, Q, R);
      solver.computeP();

      DenseMatrix64F assembledQ = new DenseMatrix64F(2, 2);
      CommonOps.multTransA(A, solver.getP(), assembledQ);
      CommonOps.multAdd(solver.getP(), A, assembledQ);

      DenseMatrix64F RInv = new DenseMatrix64F(2, 2);
      NativeCommonOps.invert(R, RInv);
      DenseMatrix64F BTransposeP = new DenseMatrix64F(2, 2);
      CommonOps.multTransA(B, solver.getP(), BTransposeP);
      DenseMatrix64F BRInv = new DenseMatrix64F(2, 2);
      CommonOps.mult(B, RInv, BRInv);
      DenseMatrix64F PBRInv = new DenseMatrix64F(2, 2);
      CommonOps.mult(solver.getP(), BRInv, PBRInv);

      DenseMatrix64F PBRInvBTransposeP = new DenseMatrix64F(2, 2);
      CommonOps.mult(PBRInv, BTransposeP, PBRInvBTransposeP);

      CommonOps.addEquals(assembledQ, -1.0, PBRInvBTransposeP);

      CommonOps.scale(-1.0, assembledQ);

      EjmlUnitTests.assertEquals(AInput, A, epsilon);
      EjmlUnitTests.assertEquals(BInput, B, epsilon);
      EjmlUnitTests.assertEquals(QInput, Q, epsilon);
      EjmlUnitTests.assertEquals(RInput, R, epsilon);

      assertSolutionIsValid(AInput, BInput, QInput, RInput, solver.getP());
   }

   @Test
   public void testMatlabCare()
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

      CARESolver solver = getSolver();
      solver.setMatrices(A, B, Q, R);
      solver.computeP();

      DenseMatrix64F PExpected = new DenseMatrix64F(2, 2);
      PExpected.set(0, 0, 0.5895);
      PExpected.set(0, 1, 1.8216);
      PExpected.set(1, 0, 1.8216);
      PExpected.set(1, 1, 8.8188);

      EjmlUnitTests.assertEquals(PExpected, solver.getP(), 1e-4);
      assertSolutionIsValid(A, B, Q, R, solver.getP());
   }

   @Test
   public void testMatlabCare2()
   {
      int n = 3;
      int m = 1;
      DenseMatrix64F A = new DenseMatrix64F(n, n);
      DenseMatrix64F B = new DenseMatrix64F(n, m);
      DenseMatrix64F C = new DenseMatrix64F(1, n);
      DenseMatrix64F Q = new DenseMatrix64F(n, n);
      DenseMatrix64F R = new DenseMatrix64F(1, 1);
      A.set(0, 0, 1);
      A.set(0, 1, -2);
      A.set(0, 2, 3);
      A.set(1, 0, -4);
      A.set(1, 1, 5);
      A.set(1, 2, 6);
      A.set(2, 0, 7);
      A.set(2, 1, 8);
      A.set(2, 2, 9);

      B.set(0, 0, 5);
      B.set(1, 0, 6);
      B.set(2, 0, -7);
      C.set(0, 0, 7);
      C.set(0, 1, -8);
      C.set(0, 2, 9);
      R.set(0, 0, 1);

      CommonOps.multInner(C, Q);

      CARESolver solver = getSolver();
      solver.setMatrices(A, B, Q, R);
      solver.computeP();

      assertSolutionIsValid(A, B, Q, R, solver.getP());
   }

   private static void assertSolutionIsValid(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, DenseMatrix64F P)
   {
      int n = A.getNumRows();
      int m = B.getNumCols();
      DenseMatrix64F assembledQ = new DenseMatrix64F(n, n);
      CommonOps.multTransA(A, P, assembledQ);
      CommonOps.multAdd(P, A, assembledQ);

      DenseMatrix64F RInv = new DenseMatrix64F(2, 2);
      NativeCommonOps.invert(R, RInv);
      DenseMatrix64F BTransposeP = new DenseMatrix64F(m, n);
      CommonOps.multTransA(B, P, BTransposeP);
      DenseMatrix64F BRInv = new DenseMatrix64F(n, m);
      CommonOps.mult(B, RInv, BRInv);
      DenseMatrix64F PBRInv = new DenseMatrix64F(n, m);
      CommonOps.mult(P, BRInv, PBRInv);

      DenseMatrix64F PBRInvBTransposeP = new DenseMatrix64F(n, n);
      CommonOps.mult(PBRInv, BTransposeP, PBRInvBTransposeP);

      CommonOps.addEquals(assembledQ, -1.0, PBRInvBTransposeP);

      CommonOps.scale(-1.0, assembledQ);

      EjmlUnitTests.assertEquals(Q, assembledQ, epsilon);
   }
}
