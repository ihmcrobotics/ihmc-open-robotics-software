package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import java.util.Random;

public class DiagonalMatrixToolsTest
{
   private final double epsilon = 1e-6;

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testSquareInvert()
   {
      Random random = new Random();
      int iters = 50;

      for (int iter = 0; iter < iters; iter++)
      {
         int size = random.nextInt(1000);
         DenseMatrix64F matrix = CommonOps.identity(size, size);
         DenseMatrix64F invMatrix = new DenseMatrix64F(size, size);
         DenseMatrix64F otherInvMatrix = new DenseMatrix64F(size, size);

         for (int index = 0; index < size; index++)
            matrix.set(index, index, 10000.0 * random.nextDouble() - 5000.0);

         LinearSolver solver = LinearSolverFactory.linear(size);
         solver.setA(matrix);
         solver.invert(invMatrix);

         DiagonalMatrixTools.invertDiagonalMatrix(matrix, otherInvMatrix);

         JUnitTools.assertMatrixEquals(otherInvMatrix, invMatrix, epsilon);

         for (int row = 0; row < size; row++)
         {
            for (int col = 0; col < size; col++)
            {
               if (row != col)
                  Assert.assertEquals(otherInvMatrix.get(row, col), 0.0, epsilon);
               else
                  Assert.assertEquals(otherInvMatrix.get(row, col), 1.0 / matrix.get(row, col), epsilon);
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testPreMult()
   {
      Random random = new Random();

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int diagonalRows = random.nextInt(1000);
         int interiorCols = random.nextInt(1000);
         int randomCols = random.nextInt(1000);

         DenseMatrix64F diagonal = CommonOps.identity(diagonalRows, interiorCols);
         DenseMatrix64F randomMatrix = new DenseMatrix64F(interiorCols, randomCols);
         DenseMatrix64F solution = new DenseMatrix64F(diagonalRows, randomCols);
         DenseMatrix64F otherSolution = new DenseMatrix64F(diagonalRows, randomCols);

         for (int row = 0; row < interiorCols; row++)
         {
            for (int col = 0; col < randomCols; col++)
            {
               randomMatrix.set(row, col, 10000.0 * random.nextDouble() - 5000.0);
            }
         }

         for (int index = 0; index < Math.min(diagonalRows, interiorCols); index++)
         {
            diagonal.set(index, index, 10000.0 * random.nextDouble() - 5000.0);
         }

         DiagonalMatrixTools.preMult(diagonal, randomMatrix, solution);
         CommonOps.mult(diagonal, randomMatrix, otherSolution);

         JUnitTools.assertMatrixEquals(solution, otherSolution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testPostMult()
   {
      Random random = new Random();

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int leadingRows = random.nextInt(1000);
         int interiorCols = random.nextInt(1000);
         int randomCols = random.nextInt(1000);

         DenseMatrix64F diagonal = CommonOps.identity(interiorCols, randomCols);
         DenseMatrix64F randomMatrix = new DenseMatrix64F(leadingRows, interiorCols);
         DenseMatrix64F solution = new DenseMatrix64F(leadingRows, randomCols);
         DenseMatrix64F otherSolution = new DenseMatrix64F(leadingRows, randomCols);

         for (int row = 0; row < leadingRows; row++)
         {
            for (int col = 0; col < interiorCols; col++)
            {
               randomMatrix.set(row, col, 10000.0 * random.nextDouble() - 5000.0);
            }
         }

         for (int index = 0; index < Math.min(randomCols, interiorCols); index++)
         {
            diagonal.set(index, index, 10000.0 * random.nextDouble() - 5000.0);
         }

         DiagonalMatrixTools.postMult(randomMatrix, diagonal, solution);
         CommonOps.mult(randomMatrix, diagonal, otherSolution);

         JUnitTools.assertMatrixEquals(solution, otherSolution, epsilon);
      }
   }
}
