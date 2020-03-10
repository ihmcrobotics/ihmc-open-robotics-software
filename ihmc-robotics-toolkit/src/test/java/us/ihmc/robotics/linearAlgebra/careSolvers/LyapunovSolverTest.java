package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
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
         DenseMatrix64F A = CARESolverTestTools.generateRandomSymmetricPDMatrix(random, size);
         DenseMatrix64F Q = CARESolverTestTools.generateRandomSymmetricPDMatrix(random, size);
         DenseMatrix64F AOriginal = new DenseMatrix64F(A);
         DenseMatrix64F QOriginal = new DenseMatrix64F(Q);

         solver.setMatrices(A, Q);
         solver.solve();

         DenseMatrix64F X = solver.getX();

         DenseMatrix64F constructedQ = new DenseMatrix64F(size, size);
         CommonOps.multTransA(A, X, constructedQ);
         CommonOps.multAdd(X, A, constructedQ);
         CommonOps.scale(-1.0, constructedQ);

         EjmlUnitTests.assertEquals(AOriginal, A, epsilon);
         EjmlUnitTests.assertEquals(QOriginal, Q, epsilon);
         EjmlUnitTests.assertEquals(constructedQ, Q, epsilon);
      }
   }

}
