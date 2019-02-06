package us.ihmc.robotics.linearAlgebra;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.testing.JUnitTools;

public class DiagonalMatrixToolsTest
{
   private final double epsilon = 1e-6;

   @Test
   public void testSquareInvert()
   {
      Random random = new Random(1738L);
      int iters = 1000;

      for (int iter = 0; iter < iters; iter++)
      {
         int size = random.nextInt(100);
         DenseMatrix64F matrix = CommonOps.identity(size, size);
         DenseMatrix64F invMatrix = new DenseMatrix64F(size, size);
         DenseMatrix64F otherInvMatrix = new DenseMatrix64F(size, size);
         DenseMatrix64F otherInvMatrixB = new DenseMatrix64F(size, size);

         for (int index = 0; index < size; index++)
            matrix.set(index, index, RandomNumbers.nextDouble(random, 10000.0));

         otherInvMatrixB.set(matrix);

         LinearSolver solver = LinearSolverFactory.linear(size);
         solver.setA(matrix);
         solver.invert(invMatrix);

         DiagonalMatrixTools.invertDiagonalMatrix(matrix, otherInvMatrix);
         DiagonalMatrixTools.invertDiagonalMatrix(otherInvMatrixB);

         JUnitTools.assertMatrixEquals(invMatrix, otherInvMatrix, epsilon);
         JUnitTools.assertMatrixEquals(invMatrix, otherInvMatrixB, epsilon);

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

         DiagonalMatrixTools.invertDiagonalMatrix(otherInvMatrix);
         JUnitTools.assertMatrixEquals(matrix, otherInvMatrix, epsilon);
      }
   }

   @Test
   public void testPreMult()
   {
      Random random = new Random(1738L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int diagonalRows = RandomNumbers.nextInt(random, 1, 100);
         int interiorCols = RandomNumbers.nextInt(random, 1, 100);
         int randomCols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(diagonalRows, interiorCols);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(diagonalRows, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(interiorCols, randomCols, -5000.0, 5000.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(diagonalRows, randomCols);
         DenseMatrix64F solutionB = new DenseMatrix64F(diagonalRows, randomCols);
         DenseMatrix64F otherSolution = new DenseMatrix64F(diagonalRows, randomCols);

         for (int index = 0; index < Math.min(diagonalRows, interiorCols); index++)
         {
            double value = RandomNumbers.nextDouble(random, 5000.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, value);
         }

         DiagonalMatrixTools.preMult(diagonal, randomMatrix, solution);
         DiagonalMatrixTools.preMult(diagonalVector, randomMatrix, solutionB);
         CommonOps.mult(diagonal, randomMatrix, otherSolution);

         JUnitTools.assertMatrixEquals(otherSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(otherSolution, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionB, epsilon);
      }
   }

   @Test
   public void testPreMultVector()
   {
      Random random = new Random(1738L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int diagonalRows = RandomNumbers.nextInt(random, 1, 100);
         int interiorCols = RandomNumbers.nextInt(random, 1, 100);
         int randomCols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(diagonalRows, interiorCols);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(diagonalRows, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(interiorCols, randomCols, -5000.0, 5000.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(diagonalRows, randomCols);
         DenseMatrix64F otherSolution = new DenseMatrix64F(diagonalRows, randomCols);

         for (int index = 0; index < Math.min(diagonalRows, interiorCols); index++)
         {
            double value = RandomNumbers.nextDouble(random, 5000.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, value);
         }

         DiagonalMatrixTools.preMult(diagonalVector, randomMatrix, solution);
         CommonOps.mult(diagonal, randomMatrix, otherSolution);

         JUnitTools.assertMatrixEquals(otherSolution, solution, epsilon);
      }
   }

   @Test
   public void testPreMultAddBlock()
   {
      Random random = new Random(1738L);

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int rows = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);
         int interiorCols = RandomNumbers.nextInt(random, 1, 100);

         int fullRows = RandomNumbers.nextInt(random, rows, 500);
         int fullCols = RandomNumbers.nextInt(random, cols, 500);

         int startRow = RandomNumbers.nextInt(random, 0, fullRows - rows);
         int startCol = RandomNumbers.nextInt(random, 0, fullCols - cols);

         double scalar = RandomNumbers.nextDouble(random, 1000.0);

         DenseMatrix64F diagonal = CommonOps.identity(rows, interiorCols);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(rows, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(interiorCols, cols, -100.0, 100.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullRows, fullCols, -10.0, 10.0, random);
         DenseMatrix64F solutionB = new DenseMatrix64F(solution);
         DenseMatrix64F solutionC = new DenseMatrix64F(solution);
         DenseMatrix64F solutionD = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(solution);

         for (int index = 0; index < Math.min(rows, interiorCols); index++)
         {
            double value = RandomNumbers.nextDouble(random, 100.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(rows, cols);
         CommonOps.mult(diagonal, randomMatrix, temp);
         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, rows, cols, 1.0);
         MatrixTools.addMatrixBlock(expectedSolutionB, startRow, startCol, temp, 0, 0, rows, cols, scalar);

         DiagonalMatrixTools.preMultAddBlock(diagonal, randomMatrix, solution, startRow, startCol);
         DiagonalMatrixTools.preMultAddBlock(scalar, diagonal, randomMatrix, solutionB, startRow, startCol);
         DiagonalMatrixTools.preMultAddBlock(diagonalVector, randomMatrix, solutionC, startRow, startCol);
         DiagonalMatrixTools.preMultAddBlock(scalar, diagonalVector, randomMatrix, solutionD, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionC, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionD, epsilon);

         JUnitTools.assertMatrixEquals(solution, solutionC, epsilon);
         JUnitTools.assertMatrixEquals(solutionB, solutionD, epsilon);
      }
   }

   @Test
   public void testPostMult()
   {
      Random random = new Random(1738L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int leadingRows = RandomNumbers.nextInt(random, 1, 100);
         int interiorCols = RandomNumbers.nextInt(random, 1, 100);
         int randomCols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(interiorCols, randomCols);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(interiorCols, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(leadingRows, interiorCols, -5000.0, 5000.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(leadingRows, randomCols);
         DenseMatrix64F solutionB = new DenseMatrix64F(leadingRows, randomCols);
         DenseMatrix64F otherSolution = new DenseMatrix64F(leadingRows, randomCols);

         for (int index = 0; index < Math.min(randomCols, interiorCols); index++)
         {
            double value = RandomNumbers.nextDouble(random, 5000.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DiagonalMatrixTools.postMult(randomMatrix, diagonal, solution);
         DiagonalMatrixTools.postMult(randomMatrix, diagonalVector, solutionB);
         CommonOps.mult(randomMatrix, diagonal, otherSolution);

         JUnitTools.assertMatrixEquals(otherSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(otherSolution, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionB, epsilon);
      }
   }

   @Test
   public void testPostMultVector()
   {
      Random random = new Random(1738L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int leadingRows = RandomNumbers.nextInt(random, 1, 100);
         int interiorCols = RandomNumbers.nextInt(random, 1, 100);
         int randomCols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(interiorCols, randomCols);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(interiorCols, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(leadingRows, interiorCols, -5000.0, 5000.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(leadingRows, randomCols);
         DenseMatrix64F otherSolution = new DenseMatrix64F(leadingRows, randomCols);

         for (int index = 0; index < Math.min(randomCols, interiorCols); index++)
         {
            double value = RandomNumbers.nextDouble(random, 5000.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DiagonalMatrixTools.postMult(randomMatrix, diagonalVector, solution);
         CommonOps.mult(randomMatrix, diagonal, otherSolution);

         JUnitTools.assertMatrixEquals(solution, otherSolution, epsilon);
      }
   }

   @Test
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

   @Test
   public void testRandomPostMultTransA()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int leadingRows = RandomNumbers.nextInt(random, 1, 100);
         int interiorCols = RandomNumbers.nextInt(random, 1, 100);
         int randomCols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(interiorCols, randomCols);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(interiorCols, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(interiorCols, leadingRows, -5000.0, 5000.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(leadingRows, randomCols);
         DenseMatrix64F solutionB = new DenseMatrix64F(leadingRows, randomCols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(leadingRows, randomCols);

         for (int index = 0; index < Math.min(randomCols, interiorCols); index++)
         {
            double value = RandomNumbers.nextDouble(random, 5000.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DiagonalMatrixTools.postMultTransA(randomMatrix, diagonal, solution);
         DiagonalMatrixTools.postMultTransA(randomMatrix, diagonalVector, solutionB);
         CommonOps.multTransA(randomMatrix, diagonal, expectedSolution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionB, epsilon);
      }
   }

   @Test
   public void testEasyMultInner()
   {
      Random random = new Random(124L);

      int iters = 1000;

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

   @Test
   public void testRandomMultInner()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);

         double diagonalScalar = RandomNumbers.nextDouble(random, 50.0);
         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, variables);
         DenseMatrix64F solutionB = new DenseMatrix64F(variables, variables);
         DenseMatrix64F solutionC = new DenseMatrix64F(variables, variables);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, variables);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(variables, variables);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);

         // Compute: H += J^T W J
         CommonOps.mult(tempJtW, randomMatrix, expectedSolution);
         CommonOps.multTransA(diagonalScalar, randomMatrix, randomMatrix, expectedSolutionB);

         DiagonalMatrixTools.multInner(randomMatrix, diagonal, solution);
         DiagonalMatrixTools.multInner(randomMatrix, diagonalScalar, solutionB);
         DiagonalMatrixTools.multInner(randomMatrix, diagonalVector, solutionC);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionC, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionC, epsilon);
      }
   }

   @Test
   public void testEasyMultOuter()
   {
      Random random = new Random(124L);

      int iters = 1000;

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

   @Test
   public void testRandomMultOuter()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);

         double diagonalScalar = RandomNumbers.nextDouble(random, 50.0);
         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(variables, taskSize, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, variables);
         DenseMatrix64F solutionB = new DenseMatrix64F(variables, variables);
         DenseMatrix64F solutionC = new DenseMatrix64F(variables, variables);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, variables);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(variables, variables);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F tempWJ = new DenseMatrix64F(variables, taskSize);
         CommonOps.mult(randomMatrix, diagonal, tempWJ);
         CommonOps.multTransB(tempWJ, randomMatrix, expectedSolution);
         CommonOps.multTransB(diagonalScalar, randomMatrix, randomMatrix, expectedSolutionB);

         DiagonalMatrixTools.multOuter(randomMatrix, diagonal, solution);
         DiagonalMatrixTools.multOuter(randomMatrix, diagonalScalar, solutionB);
         DiagonalMatrixTools.multOuter(randomMatrix, diagonalVector, solutionC);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionC, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionC, epsilon);
      }
   }

   @Test
   public void testEasyMultAddInner()
   {
      Random random = new Random(124L);

      int iters = 1000;

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

   @Test
   public void testRandomMultAddInner()
   {
      Random random = new Random(124L);

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);

         double scale = RandomNumbers.nextDouble(random, 100.0);
         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F solution = RandomMatrices.createRandom(variables, variables, -50, 50, random);
         DenseMatrix64F solutionB = new DenseMatrix64F(solution);
         DenseMatrix64F solutionD = new DenseMatrix64F(solution);
         DenseMatrix64F solutionE = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.multAdd(tempJtW, randomMatrix, expectedSolution);
         CommonOps.multAdd(scale, tempJtW, randomMatrix, expectedSolutionB);

         DiagonalMatrixTools.multAddInner(randomMatrix, diagonal, solution);
         DiagonalMatrixTools.multAddInner(scale, randomMatrix, diagonal, solutionB);
         DiagonalMatrixTools.multAddInner(randomMatrix, diagonalVector, solutionD);
         DiagonalMatrixTools.multAddInner(scale, randomMatrix, diagonalVector, solutionE);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionD, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionE, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionD, epsilon);
         JUnitTools.assertMatrixEquals(solutionB, solutionE, epsilon);
      }
   }

   @Test
   public void testRandomMultAddBlockInner()
   {
      Random random = new Random(124L);

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int fullVariables = RandomNumbers.nextInt(random, variables, 500);

         int startRow = RandomNumbers.nextInt(random, 0, fullVariables - variables);
         int startCol = RandomNumbers.nextInt(random, 0, fullVariables - variables);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F expectedSolution = RandomMatrices.createRandom(fullVariables, fullVariables, -50, 50, random);
         DenseMatrix64F solution = new DenseMatrix64F(expectedSolution);
         DenseMatrix64F solutionB = new DenseMatrix64F(expectedSolution);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);
         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonalVector, solutionB, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionB, epsilon);
      }
   }

   @Test
   public void testEasyMultAddBlockInner()
   {
      Random random = new Random(124L);

      int iters = 1000;

      // top left
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 0;
         int startRow = 0;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

      // top middle
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 1;
         int startRow = 0;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

      // top right
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 2;
         int startRow = 0;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

      // middle left
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 0;
         int startRow = 1;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

      // middle
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 1;
         int startRow = 1;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

      // middle right
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 2;
         int startRow = 1;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

      // bottom left
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 0;
         int startRow = 2;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

      // bottom middle
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 1;
         int startRow = 2;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

      // bottom right
      for (int i = 0; i < iters; i++)
      {
         int variables = 4;
         int taskSize = 3;

         int fullVariables = 6;
         int startCol = 2;
         int startRow = 2;

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullVariables, fullVariables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, RandomNumbers.nextDouble(random, 50.0));
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         DiagonalMatrixTools.multAddBlockInner(randomMatrix, diagonal, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }

   }

   @Test
   public void testEasyInnerDiagonalMult()
   {
      Random random = new Random(124L);

      int iters = 1000;

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

   @Test
   public void testRandomInnerDiagonalMult()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(variables, taskSize, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, cols);
         DenseMatrix64F solutionB = new DenseMatrix64F(variables, cols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, cols);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.mult(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMult(randomMatrixA, diagonal, randomMatrixB, solution);
         DiagonalMatrixTools.innerDiagonalMult(randomMatrixA, diagonalVector, randomMatrixB, solutionB);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionB, epsilon);
      }
   }

   @Test
   public void testRandomInnerDiagonalMultVector()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(variables, taskSize, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, cols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, cols);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.mult(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMult(randomMatrixA, diagonalVector, randomMatrixB, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @Test
   public void testEasyInnerDiagonalMultTransA()
   {
      Random random = new Random(124L);

      int iters = 1000;

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

   @Test
   public void testRandomInnerDiagonalMultTransA()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, cols);
         DenseMatrix64F solutionB = new DenseMatrix64F(variables, cols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, cols);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multTransA(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMultTransA(randomMatrixA, diagonal, randomMatrixB, solution);
         DiagonalMatrixTools.innerDiagonalMultTransA(randomMatrixA, diagonalVector, randomMatrixB, solutionB);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionB, epsilon);
      }
   }

   @Test
   public void testRandomInnerDiagonalMultTransAVector()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = new DenseMatrix64F(variables, cols);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(variables, cols);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multTransA(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMultTransA(randomMatrixA, diagonalVector, randomMatrixB, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @Test
   public void testRandomInnerDiagonalMultAddTransA()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = RandomMatrices.createRandom(variables, cols, -50.0, 50.0, random);
         DenseMatrix64F solutionB = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multAddTransA(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMultAddTransA(randomMatrixA, diagonal, randomMatrixB, solution);
         DiagonalMatrixTools.innerDiagonalMultAddTransA(randomMatrixA, diagonalVector, randomMatrixB, solutionB);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionB, epsilon);
      }
   }

   @Test
   public void testRandomInnerDiagonalMultAddTransAVector()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);
         DenseMatrix64F solution = RandomMatrices.createRandom(variables, cols, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multAddTransA(randomMatrixA, temp, expectedSolution);

         DiagonalMatrixTools.innerDiagonalMultAddTransA(randomMatrixA, diagonalVector, randomMatrixB, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
      }
   }

   @Test
   public void testRandomInnerDiagonalMultAddBlockTransA()
   {
      Random random = new Random(124L);

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int rows = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);
         int fullRows = RandomNumbers.nextInt(random, rows, 500);
         int fullCols = RandomNumbers.nextInt(random, cols, 500);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);

         int rowStart = RandomNumbers.nextInt(random, 0, fullRows - rows);
         int colStart = RandomNumbers.nextInt(random, 0, fullCols - cols);

         double scale = RandomNumbers.nextDouble(random, 1000.0);
         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, rows, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullRows, fullCols, -50.0, 50.0, random);
         DenseMatrix64F solutionB = new DenseMatrix64F(solution);
         DenseMatrix64F solutionC = new DenseMatrix64F(solution);
         DenseMatrix64F solutionD = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         DenseMatrix64F temp2 = new DenseMatrix64F(rows, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multTransA(randomMatrixA, temp, temp2);

         MatrixTools.addMatrixBlock(expectedSolution, rowStart, colStart, temp2, 0, 0, rows, cols, 1.0);
         MatrixTools.addMatrixBlock(expectedSolutionB, rowStart, colStart, temp2, 0, 0, rows, cols, scale);

         DiagonalMatrixTools.innerDiagonalMultAddBlockTransA(randomMatrixA, diagonal, randomMatrixB, solution, rowStart, colStart);
         DiagonalMatrixTools.innerDiagonalMultAddBlockTransA(randomMatrixA, diagonalVector, randomMatrixB, solutionC, rowStart, colStart);
         DiagonalMatrixTools.innerDiagonalMultAddBlockTransA(scale, randomMatrixA, diagonal, randomMatrixB, solutionB, rowStart, colStart);
         DiagonalMatrixTools.innerDiagonalMultAddBlockTransA(scale, randomMatrixA, diagonalVector, randomMatrixB, solutionD, rowStart, colStart);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolution, solutionC, epsilon);
         JUnitTools.assertMatrixEquals(solution, solutionC, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionD, epsilon);
         JUnitTools.assertMatrixEquals(solutionB, solutionD, epsilon);
      }
   }

   @Test
   public void testRandomInnerDiagonalMultAddBlockTransAVector()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int rows = RandomNumbers.nextInt(random, 1, 100);
         int cols = RandomNumbers.nextInt(random, 1, 100);
         int fullRows = RandomNumbers.nextInt(random, rows, 500);
         int fullCols = RandomNumbers.nextInt(random, cols, 500);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);

         int rowStart = RandomNumbers.nextInt(random, 0, fullRows - rows);
         int colStart = RandomNumbers.nextInt(random, 0, fullCols - cols);

         double scale = RandomNumbers.nextDouble(random, 1000.0);
         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         DenseMatrix64F diagonalVector = new DenseMatrix64F(taskSize, 1);
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, rows, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullRows, fullCols, -50.0, 50.0, random);
         DenseMatrix64F solutionB = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(solution);

         for (int index = 0; index < taskSize; index++)
         {
            double value = RandomNumbers.nextDouble(random, 50.0);
            diagonal.set(index, index, value);
            diagonalVector.set(index, 0, value);
         }

         DenseMatrix64F temp = new DenseMatrix64F(taskSize, cols);
         DenseMatrix64F temp2 = new DenseMatrix64F(rows, cols);
         CommonOps.mult(diagonal, randomMatrixB, temp);
         CommonOps.multTransA(randomMatrixA, temp, temp2);

         MatrixTools.addMatrixBlock(expectedSolution, rowStart, colStart, temp2, 0, 0, rows, cols, 1.0);
         MatrixTools.addMatrixBlock(expectedSolutionB, rowStart, colStart, temp2, 0, 0, rows, cols, scale);

         DiagonalMatrixTools.innerDiagonalMultAddBlockTransA(randomMatrixA, diagonalVector, randomMatrixB, solution, rowStart, colStart);
         DiagonalMatrixTools.innerDiagonalMultAddBlockTransA(scale, randomMatrixA, diagonalVector, randomMatrixB, solutionB, rowStart, colStart);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, epsilon);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, epsilon);
      }
   }
}

