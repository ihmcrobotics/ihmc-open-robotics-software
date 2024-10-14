package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public class LyapunovSolverTest
{
   private static final double epsilon = 1e-7;
   private static final int iters = 100;

   @Test
   public void testRandom()
   {
      LyapunovEquationSolver solver = new LyapunovEquationSolver();
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         int size = RandomNumbers.nextInt(random, 1, 10);
         DMatrixRMaj A = CARESolverTestTools.generateRandomSymmetricPDMatrix(random, size);
         DMatrixRMaj Q = CARESolverTestTools.generateRandomSymmetricPDMatrix(random, size);
         DMatrixRMaj AOriginal = new DMatrixRMaj(A);
         DMatrixRMaj QOriginal = new DMatrixRMaj(Q);

         solver.setMatrices(A, Q);
         solver.solve();

         DMatrixRMaj X = solver.getX();

         DMatrixRMaj constructedQ = new DMatrixRMaj(size, size);
         CommonOps_DDRM.multTransA(A, X, constructedQ);
         CommonOps_DDRM.multAdd(X, A, constructedQ);
         CommonOps_DDRM.scale(-1.0, constructedQ);

         EjmlUnitTests.assertEquals(AOriginal, A, epsilon);
         EjmlUnitTests.assertEquals(QOriginal, Q, epsilon);
         EjmlUnitTests.assertEquals(constructedQ, Q, epsilon);
      }
   }

}
