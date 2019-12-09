package us.ihmc.robotics.linearAlgebra.careSolver;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.KleinmanCARESolver;

public class KleinmanCARESolverTest
{
   @Test
   public void testSuperSimple()
   {
      DenseMatrix64F A = CommonOps.identity(2);
      DenseMatrix64F B = new DenseMatrix64F(2, 1);
      B.set(1, 0, 1.0);
      DenseMatrix64F Q = CommonOps.identity(2);
      DenseMatrix64F R = CommonOps.identity(1);

      KleinmanCARESolver solver = new KleinmanCARESolver(A, B, Q, R);

      DenseMatrix64F expectedQ = new DenseMatrix64F(2, 2);
      CommonOps.multTransA(A, solver.getP(), expectedQ);
      CommonOps.multAdd(solver.getP(), A, expectedQ);

      DenseMatrix64F RInv = new DenseMatrix64F(2, 2);
      NativeCommonOps.invert(R, RInv);
      DenseMatrix64F BTransposeP = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(B, solver.getP(), BTransposeP);
      DenseMatrix64F BRInv = new DenseMatrix64F(2, 1);
      CommonOps.mult(B, RInv, BRInv);
      DenseMatrix64F PBRInv = new DenseMatrix64F(2, 1);
      CommonOps.mult(solver.getP(), BRInv, PBRInv);

      DenseMatrix64F PBRInvBTransposeP = new DenseMatrix64F(2, 2);
      CommonOps.mult(PBRInv, BTransposeP, PBRInvBTransposeP);

      CommonOps.addEquals(expectedQ, -1.0, PBRInvBTransposeP);

      CommonOps.scale(-1.0, expectedQ);

      EjmlUnitTests.assertEquals(expectedQ, Q, 1e-7);
   }

}
