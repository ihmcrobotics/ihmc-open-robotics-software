package us.ihmc.robotics.linearAlgebra.careSolver;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;

public abstract class CARESolverTest
{
   private static final double epsilon = 1e-7;

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

      EjmlUnitTests.assertEquals(Q, assembledQ, epsilon);
   }
}
