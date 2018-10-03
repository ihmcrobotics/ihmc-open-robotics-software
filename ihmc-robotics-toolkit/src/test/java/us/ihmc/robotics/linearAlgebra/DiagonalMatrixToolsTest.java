package us.ihmc.robotics.linearAlgebra;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.testing.JUnitTools;

public class DiagonalMatrixToolsTest
{
   private final double epsilon = 1e-6;

   @ContinuousIntegrationTest(estimatedDuration = 14.4)
   @Test(timeout = 72000)
   public void testSquareInvert()
   {
      Random random = new Random();
      int iters = 50;

      for (int iter = 0; iter < iters; iter++)
      {
         int size = random.nextInt(100);
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

   @ContinuousIntegrationTest(estimatedDuration = 9.9)
   @Test(timeout = 50000)
   public void testPreMult()
   {
      Random random = new Random();

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int diagonalRows = random.nextInt(100);
         int interiorCols = random.nextInt(100);
         int randomCols = random.nextInt(100);

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

   @ContinuousIntegrationTest(estimatedDuration = 10.4)
   @Test(timeout = 52000)
   public void testPostMult()
   {
      Random random = new Random();

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int leadingRows = random.nextInt(100);
         int interiorCols = random.nextInt(100);
         int randomCols = random.nextInt(100);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPostMultTransA()
   {
         DenseMatrix64F diagonal = new DenseMatrix64F(2, 4);
         DenseMatrix64F A = new DenseMatrix64F(2, 3);
         DenseMatrix64F solution = new DenseMatrix64F(3, 4);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(3, 4);

         diagonal.set(0, 0, 7.0);
         diagonal.set(1, 1, 8.0);

         A.set(0, 0, 1.0);
         A.set(0, 1, 2.0);
         A.set(0, 2, 3.0);

         A.set(1, 0, 4.0);
         A.set(1, 1, 5.0);
         A.set(1, 2, 6.0);

         DiagonalMatrixTools.postMultTransA(A, diagonal, solution);
         CommonOps.multTransA(A, diagonal, expectedSolution);

         JUnitTools.assertMatrixEquals(solution, expectedSolution, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testRandomPostMultTransA()
   {
      Random random = new Random(124L);

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int leadingRows = RandomNumbers.nextInt(random, 1, 100);
         int interiorCols = RandomNumbers.nextInt(random, 1, 100);
         int randomCols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(interiorCols, randomCols);
         DenseMatrix64F randomMatrix = new DenseMatrix64F(interiorCols, leadingRows);
         DenseMatrix64F solution = new DenseMatrix64F(leadingRows, randomCols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(leadingRows, randomCols);

         for (int row = 0; row < interiorCols; row++)
         {
            for (int col = 0; col < leadingRows; col++)
            {
               randomMatrix.set(row, col, RandomNumbers.nextDouble(random, 5000.0));
            }
         }

         for (int index = 0; index < Math.min(randomCols, interiorCols); index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 5000.0));
         }

         DiagonalMatrixTools.postMultTransA(randomMatrix, diagonal, solution);
         CommonOps.multTransA(randomMatrix, diagonal, expectedSolution);

         JUnitTools.assertMatrixEquals(solution, expectedSolution, epsilon);
      }
   }



   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testEasyMultInner()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;


         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, variables);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, variables);


         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DiagonalMatrixTools.postMultTransA(randomMatrix, diagonal, tempJtW);

         // Compute: H += J^T W J
         CommonOps.mult(tempJtW, randomMatrix, expectedSolution);

         DiagonalMatrixTools.multInner(randomMatrix, diagonal, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testRandomMultInner()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);


         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, variables);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, variables);


         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);

         // Compute: H += J^T W J
         CommonOps.mult(tempJtW, randomMatrix, expectedSolution);

         DiagonalMatrixTools.multInner(randomMatrix, diagonal, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testEasyMultOuter()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;


         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(variables, taskSize, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, variables);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, variables);


         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempWJ = new DenseMatrix64F(variables, taskSize);
         CommonOps.mult(randomMatrix, diagonal, tempWJ);
         CommonOps.multTransB(tempWJ, randomMatrix, expectedSolution);

         DiagonalMatrixTools.multOuter(randomMatrix, diagonal, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testRandomMultOuter()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);


         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(variables, taskSize, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, variables);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, variables);


         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempWJ = new DenseMatrix64F(variables, taskSize);
         CommonOps.mult(randomMatrix, diagonal, tempWJ);
         CommonOps.multTransB(tempWJ, randomMatrix, expectedSolution);

         DiagonalMatrixTools.multOuter(randomMatrix, diagonal, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testEasyMultAddInner()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;


         double scale = RandomNumbers.nextDouble(random, 100.0);
         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F solution = RandomMatrices.createRandom(variables, variables, -50.0, 50.0, random);
         DenseMatrix64F solutionB = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(solution);


         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DiagonalMatrixTools.postMultTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.multAdd(tempJtW, randomMatrix, expectedSolution);
         CommonOps.multAdd(scale, tempJtW, randomMatrix, expectedSolutionB);

         DiagonalMatrixTools.multAddInner(randomMatrix, diagonal, solution);
         DiagonalMatrixTools.multAddInner(scale, randomMatrix, diagonal, solutionB);


         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testRandomMultAddInner()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);


         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = RandomMatrices.createRandom(variables, variables, -50, 50, random);
         DenseMatrix64F solution = new DenseMatrix64F(expectedSolution);


         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.multAdd(tempJtW, randomMatrix, expectedSolution);

         DiagonalMatrixTools.multAddInner(randomMatrix, diagonal, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testEasyInnerDiagonalMult()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;
         int cols = 5;


         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(variables, taskSize, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, cols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, cols);



         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.mult(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMult(randomMatrixA, diagonal, randomMatrixB, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testRandomInnerDiagonalMult()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);


         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(variables, taskSize, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, cols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, cols);



         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.mult(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMult(randomMatrixA, diagonal, randomMatrixB, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testEasyInnerDiagonalMultTransA()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;
         int cols = 5;


         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, cols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, cols);



         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multTransA(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMultTransA(randomMatrixA, diagonal, randomMatrixB, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 40000)
   public void testRandomInnerDiagonalMultTransA()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, cols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, cols);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multTransA(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMultTransA(randomMatrixA, diagonal, randomMatrixB, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.1) @Test(timeout = 40000)
   public void testRandomInnerDiagonalMultAddTransA()
   {
      Random random = new Random(124L);

      int iters = 10;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = RandomMatrices.createRandom(variables, cols, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multAddTransA(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMultAddTransA(randomMatrixA, diagonal, randomMatrixB, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }
}

