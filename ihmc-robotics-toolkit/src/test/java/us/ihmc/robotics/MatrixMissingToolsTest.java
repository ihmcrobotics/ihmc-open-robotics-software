package us.ihmc.robotics;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class MatrixMissingToolsTest
{
   private static final double EPSILON = 1.0e-9;

   @Test
   public void testSetDiagonalValues()
   {
      int iters = 100;
      DMatrixRMaj matrixToSet = new DMatrixRMaj(4, 7);
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         matrixToSet.setData(RandomNumbers.nextDoubleArray(random, 4 * 7, 100));
         DMatrixRMaj originalMatrix = new DMatrixRMaj(matrixToSet);
         double value = RandomNumbers.nextDouble(random, 10.0);
         MatrixMissingTools.setDiagonalValues(matrixToSet, value, 1, 3);

         for (int row = 0; row < 4; row++)
         {
            for (int col = 0; col < 7; col++)
            {
               if (row == 1 && col == 3)
                  assertEquals(value, matrixToSet.get(row, col), EPSILON);
               else if (row == 2 && col == 4)
                  assertEquals(value, matrixToSet.get(row, col), EPSILON);
               else if (row == 3 && col == 5)
                  assertEquals(value, matrixToSet.get(row, col), EPSILON);
               else
                  assertEquals(originalMatrix.get(row, col), matrixToSet.get(row, col), EPSILON);
            }
         }
      }
   }

   @Test
   public void testNegateUnsafe()
   {
      DMatrixRMaj matrixToNegate = new DMatrixRMaj(0, 0);
      DMatrixRMaj negatedMatrix = new DMatrixRMaj(0, 0);
      int iters = 100;
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         // reinitialize the matrix to random size and value
         int initializeRows = RandomNumbers.nextInt(random, 100, 1000);
         int initializeCols = RandomNumbers.nextInt(random, 100, 1000);
         matrixToNegate.set(RandomMatrices_DDRM.rectangle(initializeRows, initializeCols, random));
         negatedMatrix.set(RandomMatrices_DDRM.rectangle(initializeRows, initializeCols, random));

         // get the matrix to compare against
         int rows = RandomNumbers.nextInt(random, 10, 500);
         int cols = RandomNumbers.nextInt(random, 10, 500);
         DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(rows, cols, random);
         matrixToNegate.set(randomMatrix);
         negatedMatrix.reshape(rows, cols);

         // negate both ways and compare
         MatrixMissingTools.unsafe_changeSign(matrixToNegate, negatedMatrix);
         CommonOps_DDRM.scale(-1.0, randomMatrix);

         EjmlUnitTests.assertEquals(randomMatrix, negatedMatrix, EPSILON);
      }
   }

   @Test
   public void testFast2x2Inverse()
   {
      int iters = 500;
      double epsilon = 1e-8;
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         DMatrixRMaj matrix = new DMatrixRMaj(2, 2);
         DMatrixRMaj matrixInverseExpected = new DMatrixRMaj(2, 2);
         DMatrixRMaj matrixInverse = new DMatrixRMaj(2, 2);
         matrix.setData(RandomNumbers.nextDoubleArray(random, 4, 10.0));

         NativeCommonOps.invert(matrix, matrixInverseExpected);
         MatrixMissingTools.fast2x2Inverse(matrix, matrixInverse);

         MatrixTestTools.assertMatrixEquals(matrixInverseExpected, matrixInverse, epsilon);
      }
   }

   @Test
   public void testToSkewSymmetric()
   {
      int iters = 500;
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         Vector3D vectorA = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vectorB = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vectorC = new Vector3D();

         vectorC.cross(vectorA, vectorB);

         DMatrixRMaj vectorBVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj vectorCVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj vectorCActual = new DMatrixRMaj(3, 1);
         DMatrixRMaj skewVectorA = new DMatrixRMaj(3, 3);
         vectorB.get(vectorBVector);
         MatrixMissingTools.toSkewSymmetricMatrix(vectorA, skewVectorA);

         CommonOps_DDRM.mult(skewVectorA, vectorBVector, vectorCVector);
         vectorC.get(vectorCActual);

         MatrixTestTools.assertMatrixEquals(vectorCActual, vectorCVector, EPSILON);
      }
   }

   @Test
   public void testSetMatrixRows()
   {
      Random random = new Random(45348L);

      // Pass in 0 for numberOfRows -- should do nothing
      DMatrixRMaj src = RandomMatrices_DDRM.rectangle(10, 10, random);
      DMatrixRMaj dest = RandomMatrices_DDRM.rectangle(10, 10, random);
      DMatrixRMaj srcCopy = new DMatrixRMaj(src);
      DMatrixRMaj destCopy = new DMatrixRMaj(dest);
      MatrixMissingTools.setMatrixRows(src, 0, dest, 0, 0);
      assertArrayEquals(srcCopy.getData(), src.getData(), EPSILON);
      assertArrayEquals(destCopy.getData(), dest.getData(), EPSILON);

      // Number of columns don't match -- should throw exception
      src = RandomMatrices_DDRM.rectangle(10, 10, random);
      dest = RandomMatrices_DDRM.rectangle(8, 8, random);  // dest is smaller than src
      int numberOfRows = 5;
      try
      {
         MatrixMissingTools.setMatrixRows(src, 0, dest, 0, numberOfRows);
         fail("Should have thrown exception");
      }
      catch (IllegalArgumentException e)
      {
         // good
      }

      // Dest is too small -- should throw exception
      src = RandomMatrices_DDRM.rectangle(10, 10, random);
      dest = RandomMatrices_DDRM.rectangle(5, 10, random);  // dest is too small
      numberOfRows = 10;
      try
      {
         MatrixMissingTools.setMatrixRows(src, 0, dest, 0, numberOfRows);
         fail("Should have thrown exception");
      }
      catch (IllegalArgumentException e)
      {
         // good
      }

      // Src is too small -- should throw exception
      src = RandomMatrices_DDRM.rectangle(5, 10, random);  // src is too small
      dest = RandomMatrices_DDRM.rectangle(10, 10, random);
      numberOfRows = 10;
      try
      {
         MatrixMissingTools.setMatrixRows(src, 0, dest, 0, numberOfRows);
         fail("Should have thrown exception");
      }
      catch (IllegalArgumentException e)
      {
         // good
      }
   }

   @Test
   public void testSetMatrixColumns()
   {
      Random random = new Random(1738L);

      // Pass in 0 for numberOfColumns -- should do nothing
      DMatrixRMaj src = RandomMatrices_DDRM.rectangle(10, 10, random);
      DMatrixRMaj dest = RandomMatrices_DDRM.rectangle(10, 10, random);
      DMatrixRMaj srcCopy = new DMatrixRMaj(src);
      DMatrixRMaj destCopy = new DMatrixRMaj(dest);
      MatrixMissingTools.setMatrixColumns(src, 0, dest, 0, 0);
      assertArrayEquals(srcCopy.getData(), src.getData(), EPSILON);
      assertArrayEquals(destCopy.getData(), dest.getData(), EPSILON);

      // Number of rows don't match -- should throw exception
      src = RandomMatrices_DDRM.rectangle(10, 10, random);
      dest = RandomMatrices_DDRM.rectangle(8, 8, random);  // dest is smaller than src
      int numberOfColumns = 5;
      try
      {
         MatrixMissingTools.setMatrixColumns(src, 0, dest, 0, numberOfColumns);
         fail("Should have thrown exception");
      }
      catch (IllegalArgumentException e)
      {
         // good
      }

      // Dest has too few columns -- should throw exception
      src = RandomMatrices_DDRM.rectangle(10, 10, random);
      dest = RandomMatrices_DDRM.rectangle(10, 5, random);  // dest only has 5 columns
      numberOfColumns = 10;
      try
      {
         MatrixMissingTools.setMatrixColumns(src, 0, dest, 0, numberOfColumns);
         fail("Should have thrown exception");
      }
      catch (IllegalArgumentException e)
      {
         // good
      }

      // Src has too few columns -- should throw exception
      src = RandomMatrices_DDRM.rectangle(10, 5, random);  // src only has 5 columns
      dest = RandomMatrices_DDRM.rectangle(10, 10, random);
      numberOfColumns = 10;
      try
      {
         MatrixMissingTools.setMatrixColumns(src, 0, dest, 0, numberOfColumns);
         fail("Should have thrown exception");
      }
      catch (IllegalArgumentException e)
      {
         // good
      }
   }

   @Test
   public void testSetSelectedMatrixDiagonals()
   {
      Random random = new Random(1738L);

      int iters = 100;

      // Nominal
      for (int i = 0; i < iters; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         DMatrixRMaj matrix = RandomMatrices_DDRM.diagonal(rowSize, columnSize, 0, 1, random);
         int indicesSize = Math.min(rowSize, columnSize);
         int[] indices = new int[indicesSize];
         for (int j = 0; j < indicesSize; ++j)
            indices[j] = random.nextInt(0, Math.min(rowSize, columnSize) - 1);
         double value = random.nextDouble();
         MatrixMissingTools.setSelectedMatrixDiagonals(indices, value, matrix);
         for (int index : indices)
            assertEquals(value, matrix.get(index, index), EPSILON);
      }

      // Pass index arrays that have incorrect sizes
      for (int i = 0; i < iters; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         DMatrixRMaj matrix = RandomMatrices_DDRM.diagonal(rowSize, columnSize, 0, 1, random);

         // Coin flip to determine if we are going to pass in a zero-size or too large index array
         if (random.nextBoolean())  // zero-size
         {
            int indicesSize = 0;
            int[] indices = new int[indicesSize];
            try
            {
               MatrixMissingTools.setSelectedMatrixDiagonals(indices, random.nextDouble(), matrix);
               fail("Should have thrown exception");
            }
            catch (IllegalArgumentException e)
            {
               // good
            }
         }
         else  // too large
         {
            int indicesSize = Math.min(rowSize, columnSize) + 1;
            int[] indices = new int[indicesSize];
            for (int j = 0; j < indicesSize; ++j)
               indices[j] = random.nextInt(0, Math.min(rowSize, columnSize) - 1);
            try
            {
               MatrixMissingTools.setSelectedMatrixDiagonals(indices, random.nextDouble(), matrix);
               fail("Should have thrown exception");
            }
            catch (IllegalArgumentException e)
            {
               // good
            }
         }
      }

      // Pass correctly-sized arrays, but with negative or too large entry values
      for (int i = 0; i < iters; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         DMatrixRMaj matrix = RandomMatrices_DDRM.diagonal(rowSize, columnSize, 0, 1, random);

         int indicesSize = Math.min(rowSize, columnSize);
         int[] indices = new int[indicesSize];
         for (int j = 0; j < indicesSize; ++j)  // start off with good indices
            indices[j] = random.nextInt(0, Math.min(rowSize, columnSize) - 1);

         // Coin flip to determine if we are going to pass in a negative or too large index
         if (random.nextBoolean())  // negative
         {
            indices[random.nextInt(0, indicesSize - 1)] = -random.nextInt(1, 10);
            try
            {
               MatrixMissingTools.setSelectedMatrixDiagonals(indices, random.nextDouble(), matrix);
               fail("Should have thrown exception");
            }
            catch (IllegalArgumentException e)
            {
               // good
            }
         }
         else  // too large
         {
            indices[random.nextInt(indicesSize)] = Math.max(rowSize, columnSize) + 1;
            try
            {
               MatrixMissingTools.setSelectedMatrixDiagonals(indices, random.nextDouble(), matrix);
               fail("Should have thrown exception");
            }
            catch (IllegalArgumentException e)
            {
               // good
            }
         }
      }
   }

   @Test
   public void testPowerCalculation()
   {
      Random random = new Random(1738L);

      double error = 1e-5;
      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int matrixSize = random.nextInt(10);
         DMatrixRMaj matrix = RandomMatrices_DDRM.rectangle(matrixSize, matrixSize, random);
         DMatrixRMaj matrixPowerTest = new DMatrixRMaj(matrixSize, matrixSize);
         int powerNumber = random.nextInt(1, 15);

         MatrixMissingTools.power(matrix, powerNumber, matrixPowerTest);

         DMatrixRMaj matrixResult = new DMatrixRMaj(matrixSize, matrixSize);
         DMatrixRMaj matrixIdentity = new DMatrixRMaj(matrixSize, matrixSize);

         ///Testing for the identity Matrix
         CommonOps_DDRM.setIdentity(matrixResult);
         CommonOps_DDRM.setIdentity(matrixIdentity);

         MatrixMissingTools.power(matrixIdentity, powerNumber, matrixResult);
         MatrixTestTools.assertMatrixEquals(matrixIdentity, matrixResult, error);

         //Testing for the Diagonal Matrix
         DMatrixRMaj matrixDiagonal = new DMatrixRMaj(matrixSize, matrixSize);
         DMatrixRMaj matrixDiagonalHand = new DMatrixRMaj(matrixSize, matrixSize);
         DMatrixRMaj matrixDiagonalResult = new DMatrixRMaj(matrixSize, matrixSize);
         CommonOps_DDRM.setIdentity(matrixDiagonal);
         double diagonalTemp = 0;
         for (int j = 0; j < matrixSize; j++)
         {
            matrixDiagonal.set(j, j, random.nextDouble(50));
            diagonalTemp = matrixDiagonal.get(j, j);
            for (int k = 1; k < powerNumber; k++)
            {
               diagonalTemp *= matrixDiagonal.get(j, j);
            }
            matrixDiagonalHand.set(j, j, diagonalTemp);
         }
         MatrixMissingTools.power(matrixDiagonal, powerNumber, matrixDiagonalResult);
         MatrixTestTools.assertMatrixEquals(matrixDiagonalHand, matrixDiagonalResult, error);

         // Test for skew matrix, simple 2x2 size of skew matrix
         DMatrixRMaj skewMatrix = new DMatrixRMaj(2,2);
         double skewTempValue = random.nextDouble();
         skewMatrix.zero();
         skewMatrix.set(0,1, skewTempValue);
         skewMatrix.set(1,0,-skewTempValue);

         DMatrixRMaj skewMatrixHand = new DMatrixRMaj(2,2);
         DMatrixRMaj skewMatrixResult = new DMatrixRMaj(2,2);

         //check square times
         skewMatrixHand.zero();
         skewMatrixHand.set(0,0, -skewTempValue*skewTempValue);
         skewMatrixHand.set(1,1, -skewTempValue*skewTempValue);
         MatrixMissingTools.power(skewMatrix,2,skewMatrixResult);
         MatrixTestTools.assertMatrixEquals(skewMatrixHand, skewMatrixResult, error);

         //check triple times
         skewMatrixHand.zero();
         skewMatrixHand.set(0,1, -skewTempValue*skewTempValue*skewTempValue);
         skewMatrixHand.set(1,0, skewTempValue*skewTempValue*skewTempValue);
         MatrixMissingTools.power(skewMatrix,3,skewMatrixResult);
         MatrixTestTools.assertMatrixEquals(skewMatrixHand, skewMatrixResult, error);

         //Check quadruple time
         skewMatrixHand.zero();
         skewMatrixHand.set(0,0, skewTempValue*skewTempValue*skewTempValue*skewTempValue);
         skewMatrixHand.set(1,1, skewTempValue*skewTempValue*skewTempValue*skewTempValue);
         MatrixMissingTools.power(skewMatrix,4,skewMatrixResult);
         MatrixTestTools.assertMatrixEquals(skewMatrixHand, skewMatrixResult, error);



         //         MatrixTestTools.assertMatrixEquals(matrixPowerTest, matrixCompareResult, error);
      }
   }

   @Test
   public void testElementWiseLessThan()
   {
      Random random = new Random(41584L);

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);

         DMatrixRMaj matrixLess = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         DMatrixRMaj matrixMore = new DMatrixRMaj(rowSize, columnSize);
         for (int j = 0; j < rowSize; j++)
         {
            for (int k = 0; k < columnSize; k++)
            {
               matrixMore.set(j, k, matrixLess.get(j, k) + random.nextDouble(0.01, 0.05));
            }
         }

         assertTrue(MatrixMissingTools.elementWiseLessThan(matrixLess, matrixMore));
         assertFalse(MatrixMissingTools.elementWiseLessThan(matrixMore, matrixLess));

         for (int j = 0; j < matrixLess.getNumElements(); j++)
         {
            DMatrixRMaj matrixLessNotAll = new DMatrixRMaj(matrixLess);
            matrixLessNotAll.set(j, matrixMore.get(j) + 0.001);
            assertFalse(MatrixMissingTools.elementWiseLessThan(matrixLess, matrixLessNotAll));
         }
      }

      assertThrows(IllegalArgumentException.class, () -> MatrixMissingTools.elementWiseLessThan(new DMatrixRMaj(1, 2), new DMatrixRMaj(2, 2)));
   }
}