package us.ihmc.robotics.linearAlgebra;

import static us.ihmc.robotics.Assert.*;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.testing.JUnitTools;

public class MatrixToolsTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testSetToNaNDenseMatrix()
   {
      DenseMatrix64F test = new DenseMatrix64F(3, 3);
      MatrixTools.setToNaN(test);

      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            assertTrue(Double.isNaN(test.get(i, j)));
         }
      }
   }

   @Test
   public void testSetToZeroDenseMatrix()
   {
      DenseMatrix64F test = new DenseMatrix64F(3, 3);
      MatrixTools.setToZero(test);

      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            assertEquals(0.0, test.get(i, j), 1e-34);
         }
      }
   }

   @Test
   public void testSetMatrixColumnFromArrayDenseMatrix()
   {
      DenseMatrix64F test = new DenseMatrix64F(2, 2);

      double[] col = new double[] {3.0, 4.0};

      MatrixTools.setMatrixColumnFromArray(test, 1, col);

      assertEquals(col[0], test.get(0, 1), 1e-8);
      assertEquals(col[1], test.get(1, 1), 1e-8);

   }

   @Test
   public void testSetMatrixFromOneBasedArrayDenseMatrix()
   {
      DenseMatrix64F test = new DenseMatrix64F(2, 1);

      double[] col = new double[] {0.0, 3.0, 4.0};

      MatrixTools.setMatrixFromOneBasedArray(test, col);

      assertEquals(col[1], test.get(0, 0), 1e-8);
      assertEquals(col[2], test.get(1, 0), 1e-8);

   }

   @Test
   public void testDiffDenseMatrixIntIntDenseMatrix()
   {
      double[][] vals = new double[][] {{1.0}, {2.0}, {4.0}, {8.0}, {16.0}, {32.0}};
      DenseMatrix64F test = new DenseMatrix64F(vals);

      DenseMatrix64F res = new DenseMatrix64F(2, 1);

      MatrixTools.diff(test, 2, 3, res);

      assertEquals(4.0, res.get(0, 0), 1e-8);
      assertEquals(8.0, res.get(1, 0), 1e-8);

   }

   @Test
   public void testDiffDoubleArrayDenseMatrix()
   {
      double[] vals = new double[] {1.0, 3.0, 4.0, 9.0, 16.0, 32.0};
      double[] expected = new double[] {2.0, 1.0, 5.0, 7.0, 16.0};
      DenseMatrix64F res = new DenseMatrix64F(5, 1);

      MatrixTools.diff(vals, res);

      for (int i = 0; i < 5; i++)
      {
         assertEquals(expected[i], res.get(i, 0), 1e-8);
      }
   }

   @Test
   public void tranformSe3IntoTransform3D()
   {
      Se3_F64 a = new Se3_F64();
      ConvertRotation3D_F64.eulerToMatrix(EulerType.XYZ, 0.1, -0.5, 1.2, a.getR());
      a.getT().set(3.3, 1.2, -9);

      RigidBodyTransform b = new RigidBodyTransform();
      MatrixTools.tranformSe3IntoTransform3D(a, b);

      Point3D_F64 p0 = new Point3D_F64(-1, 2, 3);
      Point3D p1 = new Point3D(p0.x, p0.y, p0.z);

      SePointOps_F64.transform(a, p0, p0);

      b.transform(p1);

      assertEquals(p0.x, p1.getX(), 1e-8);
      assertEquals(p0.y, p1.getY(), 1e-8);
      assertEquals(p0.z, p1.getZ(), 1e-8);
   }

   @Test
   public void testRemoveRow()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 20; i++)
      {
         int numRows = RandomNumbers.nextInt(random, 1, 100);
         int numCols = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         int indexOfRowToRemove = RandomNumbers.nextInt(random, 0, randomMatrix.getNumRows() - 1);
         DenseMatrix64F expectedMatrix = new DenseMatrix64F(numRows - 1, numCols);

         for (int rowIndex = 0; rowIndex < numRows - 1; rowIndex++)
         {
            for (int colIndex = 0; colIndex < numCols; colIndex++)
            {
               if (rowIndex >= indexOfRowToRemove)
                  expectedMatrix.set(rowIndex, colIndex, randomMatrix.get(rowIndex + 1, colIndex));
               else
                  expectedMatrix.set(rowIndex, colIndex, randomMatrix.get(rowIndex, colIndex));
            }
         }

         DenseMatrix64F matrixToTest = new DenseMatrix64F(randomMatrix);
         MatrixTools.removeRow(matrixToTest, indexOfRowToRemove);

         boolean areMatricesEqual = MatrixFeatures.isEquals(expectedMatrix, matrixToTest, 1.0e-10);
         assertTrue(areMatricesEqual);
      }
   }

   @Test
   public void testSetRow()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < 20; i++)
      {
         int numRows = RandomNumbers.nextInt(random, 1, 100);
         int numCols = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         DenseMatrix64F randomRow = RandomMatrices.createRandom(1, numCols, 1.0, 100.0, random);
         int indexOfRowToSet = RandomNumbers.nextInt(random, 0, numRows - 1);

         DenseMatrix64F expectedMatrix = new DenseMatrix64F(randomMatrix);
         DenseMatrix64F matrixToTest = new DenseMatrix64F(randomMatrix);

         for (int j = 0; j < numCols; j++)
            expectedMatrix.set(indexOfRowToSet, j, randomRow.get(0, j));

         MatrixTools.setRow(randomRow, indexOfRowToSet, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);

         numRows = RandomNumbers.nextInt(random, 1, 100);
         numCols = RandomNumbers.nextInt(random, 1, 100);
         randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         randomRow = RandomMatrices.createRandom(1, numCols, 1.0, 100.0, random);
         indexOfRowToSet = RandomNumbers.nextInt(random, 0, numRows - 1);
         double randomMultiplier = RandomNumbers.nextDouble(random, 1, 100);

         expectedMatrix = new DenseMatrix64F(randomMatrix);
         matrixToTest = new DenseMatrix64F(randomMatrix);
         for (int j = 0; j < numCols; j++)
            expectedMatrix.set(indexOfRowToSet, j, randomMultiplier * randomRow.get(0, j));

         MatrixTools.setRow(randomMultiplier, randomRow, indexOfRowToSet, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);

         numRows = RandomNumbers.nextInt(random, 1, 100);
         int numOriginRows = RandomNumbers.nextInt(random, 1, 100);
         numCols = RandomNumbers.nextInt(random, 1, 100);
         randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         randomRow = RandomMatrices.createRandom(numOriginRows, numCols, 1.0, 100.0, random);
         indexOfRowToSet = RandomNumbers.nextInt(random, 0, numRows - 1);
         int indexOfOriginRow = RandomNumbers.nextInt(random, 0, numOriginRows - 1);

         expectedMatrix = new DenseMatrix64F(randomMatrix);
         matrixToTest = new DenseMatrix64F(randomMatrix);
         for (int j = 0; j < numCols; j++)
            expectedMatrix.set(indexOfRowToSet, j, randomRow.get(indexOfOriginRow, j));

         MatrixTools.setRow(indexOfOriginRow, randomRow, indexOfRowToSet, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);

         numRows = RandomNumbers.nextInt(random, 1, 100);
         numOriginRows = RandomNumbers.nextInt(random, 1, 100);
         numCols = RandomNumbers.nextInt(random, 1, 100);
         randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         randomRow = RandomMatrices.createRandom(numOriginRows, numCols, 1.0, 100.0, random);
         indexOfRowToSet = RandomNumbers.nextInt(random, 0, numRows - 1);
         indexOfOriginRow = RandomNumbers.nextInt(random, 0, numOriginRows - 1);

         randomMultiplier = RandomNumbers.nextDouble(random, 1, 100);

         expectedMatrix = new DenseMatrix64F(randomMatrix);
         matrixToTest = new DenseMatrix64F(randomMatrix);
         for (int j = 0; j < numCols; j++)
            expectedMatrix.set(indexOfRowToSet, j, randomMultiplier * randomRow.get(indexOfOriginRow, j));

         MatrixTools.setRow(indexOfOriginRow, randomMultiplier, randomRow, indexOfRowToSet, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);



         numRows = RandomNumbers.nextInt(random, 1, 100);
         numOriginRows = RandomNumbers.nextInt(random, 1, 100);
         numCols = RandomNumbers.nextInt(random, 1, 100);
         randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         randomRow = RandomMatrices.createRandom(numOriginRows, numCols, 1.0, 100.0, random);

         int numOfRowsToSet = RandomNumbers.nextInt(random, 1, Math.min(numOriginRows, numRows));
         int[] originRowIndices = RandomNumbers.nextIntArray(random, numOfRowsToSet, 1, numOriginRows - 1);
         int[] destRowIndices = RandomNumbers.nextIntArray(random, numOfRowsToSet, 1, numRows - 1);

         expectedMatrix = new DenseMatrix64F(randomMatrix);
         matrixToTest = new DenseMatrix64F(randomMatrix);
         for (int j = 0; j < numCols; j++)
         {
            for (int k = 0; k < numOfRowsToSet; k++)
               expectedMatrix.set(destRowIndices[k], j, randomRow.get(originRowIndices[k], j));
         }

         MatrixTools.setRows(originRowIndices, randomRow, destRowIndices, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);
      }
   }


   @Test
   public void testAddRow()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < 20; i++)
      {
         int numRows = RandomNumbers.nextInt(random, 1, 100);
         int numCols = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         DenseMatrix64F randomRow = RandomMatrices.createRandom(1, numCols, 1.0, 100.0, random);
         int indexOfRowToAdd = RandomNumbers.nextInt(random, 0, numRows - 1);

         DenseMatrix64F expectedMatrix = new DenseMatrix64F(randomMatrix);
         DenseMatrix64F matrixToTest = new DenseMatrix64F(randomMatrix);

         for (int j = 0; j < numCols; j++)
            expectedMatrix.add(indexOfRowToAdd, j, randomRow.get(0, j));

         MatrixTools.addRow(randomRow, indexOfRowToAdd, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);

         numRows = RandomNumbers.nextInt(random, 1, 100);
         numCols = RandomNumbers.nextInt(random, 1, 100);
         randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         randomRow = RandomMatrices.createRandom(1, numCols, 1.0, 100.0, random);
         indexOfRowToAdd = RandomNumbers.nextInt(random, 0, numRows - 1);
         double randomMultiplier = RandomNumbers.nextDouble(random, 1, 100);

         expectedMatrix = new DenseMatrix64F(randomMatrix);
         matrixToTest = new DenseMatrix64F(randomMatrix);
         for (int j = 0; j < numCols; j++)
            expectedMatrix.add(indexOfRowToAdd, j, randomMultiplier * randomRow.get(0, j));

         MatrixTools.addRow(randomMultiplier, randomRow, indexOfRowToAdd, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);

         numRows = RandomNumbers.nextInt(random, 1, 100);
         int numOriginRows = RandomNumbers.nextInt(random, 1, 100);
         numCols = RandomNumbers.nextInt(random, 1, 100);
         randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         randomRow = RandomMatrices.createRandom(numOriginRows, numCols, 1.0, 100.0, random);
         indexOfRowToAdd = RandomNumbers.nextInt(random, 0, numRows - 1);
         int indexOfOriginRow = RandomNumbers.nextInt(random, 0, numOriginRows - 1);

         expectedMatrix = new DenseMatrix64F(randomMatrix);
         matrixToTest = new DenseMatrix64F(randomMatrix);
         for (int j = 0; j < numCols; j++)
            expectedMatrix.add(indexOfRowToAdd, j, randomRow.get(indexOfOriginRow, j));

         MatrixTools.addRow(indexOfOriginRow, randomRow, indexOfRowToAdd, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);

         numRows = RandomNumbers.nextInt(random, 1, 100);
         numOriginRows = RandomNumbers.nextInt(random, 1, 100);
         numCols = RandomNumbers.nextInt(random, 1, 100);
         randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         randomRow = RandomMatrices.createRandom(numOriginRows, numCols, 1.0, 100.0, random);
         indexOfRowToAdd = RandomNumbers.nextInt(random, 0, numRows - 1);
         indexOfOriginRow = RandomNumbers.nextInt(random, 0, numOriginRows - 1);

         randomMultiplier = RandomNumbers.nextDouble(random, 1, 100);

         expectedMatrix = new DenseMatrix64F(randomMatrix);
         matrixToTest = new DenseMatrix64F(randomMatrix);
         for (int j = 0; j < numCols; j++)
            expectedMatrix.add(indexOfRowToAdd, j, randomMultiplier * randomRow.get(indexOfOriginRow, j));

         MatrixTools.addRow(indexOfOriginRow, randomMultiplier, randomRow, indexOfRowToAdd, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);



         numRows = RandomNumbers.nextInt(random, 1, 100);
         numOriginRows = RandomNumbers.nextInt(random, 1, 100);
         numCols = RandomNumbers.nextInt(random, 1, 100);
         randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         randomRow = RandomMatrices.createRandom(numOriginRows, numCols, 1.0, 100.0, random);

         int numOfRowsToSet = RandomNumbers.nextInt(random, 1, Math.min(numOriginRows, numRows));
         int[] originRowIndices = RandomNumbers.nextIntArray(random, numOfRowsToSet, 1, numOriginRows - 1);
         int[] destRowIndices = RandomNumbers.nextIntArray(random, numOfRowsToSet, 1, numRows - 1);

         expectedMatrix = new DenseMatrix64F(randomMatrix);
         matrixToTest = new DenseMatrix64F(randomMatrix);
         for (int j = 0; j < numCols; j++)
         {
            for (int k = 0; k < numOfRowsToSet; k++)
               expectedMatrix.add(destRowIndices[k], j, randomRow.get(originRowIndices[k], j));
         }

         MatrixTools.addRows(originRowIndices, randomRow, destRowIndices, matrixToTest);
         JUnitTools.assertMatrixEquals(expectedMatrix, matrixToTest, 1.0e-10);
      }
   }

   @Test
   public void testRemoveColumn()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 20; i++)
      {
         int numRows = RandomNumbers.nextInt(random, 1, 100);
         int numCols = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         int indexOfColumnToRemove = RandomNumbers.nextInt(random, 0, randomMatrix.getNumCols() - 1);
         DenseMatrix64F expectedMatrix = new DenseMatrix64F(numRows, numCols - 1);

         for (int colIndex = 0; colIndex < numCols - 1; colIndex++)
         {
            for (int rowIndex = 0; rowIndex < numRows; rowIndex++)
            {
               if (colIndex >= indexOfColumnToRemove)
                  expectedMatrix.set(rowIndex, colIndex, randomMatrix.get(rowIndex, colIndex + 1));
               else
                  expectedMatrix.set(rowIndex, colIndex, randomMatrix.get(rowIndex, colIndex));
            }
         }

         DenseMatrix64F matrixToTest = new DenseMatrix64F(randomMatrix);
         MatrixTools.removeColumn(matrixToTest, indexOfColumnToRemove);

         for (int colIndex = 0; colIndex < numCols - 1; colIndex++)
         {
            DenseMatrix64F expectedMatrixColumn = new DenseMatrix64F(numRows, 1);
            DenseMatrix64F randomMatrixColumn = new DenseMatrix64F(numRows, 1);

            int originalColumnIndex = colIndex;
            if (colIndex >= indexOfColumnToRemove)
               originalColumnIndex++;

            CommonOps.extractColumn(expectedMatrix, colIndex, expectedMatrixColumn);
            CommonOps.extractColumn(randomMatrix, originalColumnIndex, randomMatrixColumn);
            boolean areMatricesEqual = MatrixFeatures.isEquals(randomMatrixColumn, expectedMatrixColumn, 1.0e-10);
            assertTrue(areMatricesEqual);
         }

         boolean areMatricesEqual = MatrixFeatures.isEquals(expectedMatrix, matrixToTest, 1.0e-10);
         assertTrue(areMatricesEqual);
      }
   }

   @Test
   public void testRemoveZeroRows()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 200; i++)
      {
         int numRows = RandomNumbers.nextInt(random, 1, 100);
         int numCols = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         int randomNumberOfZeroRows = RandomNumbers.nextInt(random, 0, 5);
         int[] indicesOfZeroRows = RandomNumbers.nextIntArray(random, randomNumberOfZeroRows, 0, randomMatrix.getNumRows() - 1);

         // Switching to a set to remove duplicates
         HashSet<Integer> filterForDuplicate = new HashSet<>();
         for (int zeroRowIndex : indicesOfZeroRows)
            filterForDuplicate.add(zeroRowIndex);

         indicesOfZeroRows = new int[filterForDuplicate.size()];
         int counter = 0;
         for (int filteredZeroRow : filterForDuplicate)
            indicesOfZeroRows[counter++] = filteredZeroRow;

         Arrays.sort(indicesOfZeroRows);

         for (int zeroRowIndex : indicesOfZeroRows)
         {
            for (int columnIndex = 0; columnIndex < numCols; columnIndex++)
            {
               randomMatrix.set(zeroRowIndex, columnIndex, 0.0);
            }
         }
         DenseMatrix64F expectedMatrix = new DenseMatrix64F(randomMatrix);
         for (int j = indicesOfZeroRows.length - 1; j >= 0; j--)
            MatrixTools.removeRow(expectedMatrix, indicesOfZeroRows[j]);

         DenseMatrix64F matrixToTest = new DenseMatrix64F(randomMatrix);
         MatrixTools.removeZeroRows(matrixToTest, 1.0e-12);

         boolean areMatricesEqual = MatrixFeatures.isEquals(expectedMatrix, matrixToTest, 1.0e-10);
         assertTrue(areMatricesEqual);
      }
   }

   @Test
   public void testScaleTranspose() throws Exception
   {
      Random random = new Random(165156L);
      for (int i = 0; i < 200; i++)
      {
         int numRows = RandomNumbers.nextInt(random, 1, 100);
         int numCols = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         double randomAlpha = RandomNumbers.nextDouble(random, 100.0);
         DenseMatrix64F expectedMatrix = new DenseMatrix64F(numCols, numRows);
         DenseMatrix64F actualMatrix = new DenseMatrix64F(numCols, numRows);

         CommonOps.transpose(randomMatrix, expectedMatrix);
         CommonOps.scale(randomAlpha, expectedMatrix);

         MatrixTools.scaleTranspose(randomAlpha, randomMatrix, actualMatrix);

         boolean areMatricesEqual = MatrixFeatures.isEquals(expectedMatrix, actualMatrix, 1.0e-10);
         assertTrue(areMatricesEqual);
      }
   }

   @Test
   public void testInsertFrameTupleIntoEJMLVector()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 1000; i++)
      {
         int numRows = RandomNumbers.nextInt(random, 3, 100);
         DenseMatrix64F matrixToTest = RandomMatrices.createRandom(numRows, 1, 1.0, 100.0, random);
         FramePoint3D framePointToInsert = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 100.0, 100.0, 100.0);
         int startRowToInsertFrameTuple = RandomNumbers.nextInt(random, 0, numRows - 3);
         MatrixTools.insertFrameTupleIntoEJMLVector(framePointToInsert, matrixToTest, startRowToInsertFrameTuple);

         assertEquals(framePointToInsert.getX(), matrixToTest.get(startRowToInsertFrameTuple + 0, 0), 1.0e-10);
         assertEquals(framePointToInsert.getY(), matrixToTest.get(startRowToInsertFrameTuple + 1, 0), 1.0e-10);
         assertEquals(framePointToInsert.getZ(), matrixToTest.get(startRowToInsertFrameTuple + 2, 0), 1.0e-10);
      }
   }

   @Test
   public void testExtractFrameTupleFromEJMLVector()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 1000; i++)
      {
         int numRows = RandomNumbers.nextInt(random, 3, 100);
         DenseMatrix64F matrixToExtractFrom = RandomMatrices.createRandom(numRows, 1, 1.0, 100.0, random);
         FramePoint3D framePointToTest = new FramePoint3D(null, -1.0, -1.0, -1.0);
         int startRowToExtractFrameTuple = RandomNumbers.nextInt(random, 0, numRows - 3);
         MatrixTools.extractFrameTupleFromEJMLVector(framePointToTest, matrixToExtractFrom, ReferenceFrame.getWorldFrame(), startRowToExtractFrameTuple);

         assertEquals(framePointToTest.getReferenceFrame(), ReferenceFrame.getWorldFrame());
         assertEquals(framePointToTest.getX(), matrixToExtractFrom.get(startRowToExtractFrameTuple + 0, 0), 1.0e-10);
         assertEquals(framePointToTest.getY(), matrixToExtractFrom.get(startRowToExtractFrameTuple + 1, 0), 1.0e-10);
         assertEquals(framePointToTest.getZ(), matrixToExtractFrom.get(startRowToExtractFrameTuple + 2, 0), 1.0e-10);
      }
   }

   @Test
   public void testCheckDenseMatrixDimensions()
   {
      Random ran = new Random(124L);

      int nTests = 500;
      int maxRows = 1000;
      int maxColumns = 1000;
      for (int i = 0; i < nTests; i++)
      {
         int rows = ran.nextInt(maxRows);
         int columns = ran.nextInt(maxColumns);

         // these should not throw exceptions
         try
         {
            DenseMatrix64F testm = new DenseMatrix64F(rows, columns);
            MatrixTools.checkMatrixDimensions(testm, rows, columns);
         }
         catch (Throwable e)
         {
            fail();
         }
      }

   }

   public void testMultAddBlockTransA()
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
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(taskSize, rows, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullRows, fullCols, -50.0, 50.0, random);
         DenseMatrix64F solutionB = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(solution);

         DenseMatrix64F temp = new DenseMatrix64F(rows, cols);
         CommonOps.multTransA(randomMatrixA, randomMatrixB, temp);
         MatrixTools.addMatrixBlock(expectedSolution, rowStart, colStart, temp, 0, 0, rows, cols, 1.0);
         MatrixTools.addMatrixBlock(expectedSolutionB, rowStart, colStart, temp, 0, 0, rows, cols, scale);

         MatrixTools.multAddBlockTransA(randomMatrixA, randomMatrixB, solution, rowStart, colStart);
         MatrixTools.multAddBlockTransA(scale, randomMatrixA, randomMatrixB, solutionB, rowStart, colStart);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, 1e-6);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, 1e-6);
      }
   }

   public void testMultAddBlock()
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
         DenseMatrix64F randomMatrixA = RandomMatrices.createRandom(rows, taskSize, -50.0, 50.0, random);
         DenseMatrix64F randomMatrixB = RandomMatrices.createRandom(taskSize, cols, -50.0, 50.0, random);

         DenseMatrix64F solution = RandomMatrices.createRandom(fullRows, fullCols, -50.0, 50.0, random);
         DenseMatrix64F solutionB = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);
         DenseMatrix64F expectedSolutionB = new DenseMatrix64F(solution);

         DenseMatrix64F temp = new DenseMatrix64F(rows, cols);
         CommonOps.mult(randomMatrixA, randomMatrixB, temp);
         MatrixTools.addMatrixBlock(expectedSolution, rowStart, colStart, temp, 0, 0, rows, cols, 1.0);
         MatrixTools.addMatrixBlock(expectedSolutionB, rowStart, colStart, temp, 0, 0, rows, cols, scale);

         MatrixTools.multAddBlock(randomMatrixA, randomMatrixB, solution, rowStart, colStart);
         MatrixTools.multAddBlock(scale, randomMatrixA, randomMatrixB, solutionB, rowStart, colStart);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, 1e-6);
         JUnitTools.assertMatrixEquals(expectedSolutionB, solutionB, 1e-6);
      }
   }

   @Test
   public void testRandomMultAddBlockInnerWithScalar()
   {
      Random random = new Random(124L);

      int iters = 1000;

      for (int i = 0; i < iters; i++)
      {
         int variables = RandomNumbers.nextInt(random, 1, 100);
         int taskSize = RandomNumbers.nextInt(random, 1, 100);
         int fullVariables = RandomNumbers.nextInt(random, variables, 500);

         int startRow = RandomNumbers.nextInt(random, 0, fullVariables - variables);
         int startCol = RandomNumbers.nextInt(random, 0, fullVariables - variables);

         DenseMatrix64F diagonal = CommonOps.identity(taskSize, taskSize);
         double diagonalValue = RandomNumbers.nextDouble(random, 50.0);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);

         DenseMatrix64F expectedSolution = RandomMatrices.createRandom(fullVariables, fullVariables, -50, 50, random);
         DenseMatrix64F solution = new DenseMatrix64F(expectedSolution);

         for (int index = 0; index < taskSize; index++)
         {
            diagonal.set(index, index, diagonalValue);
         }

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         DenseMatrix64F temp = new DenseMatrix64F(variables, variables);
         CommonOps.multTransA(randomMatrix, diagonal, tempJtW);
         CommonOps.mult(tempJtW, randomMatrix, temp);

         MatrixTools.addMatrixBlock(expectedSolution, startRow, startCol, temp, 0, 0, variables, variables, 1.0);

         MatrixTools.multAddBlockInner(diagonalValue, randomMatrix, solution, startRow, startCol);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, 1e-6);
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

         double diagonalScalar = RandomNumbers.nextDouble(random, 100.0);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F solution = RandomMatrices.createRandom(variables, variables, -50.0, 50.0, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);

         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         CommonOps.transpose(randomMatrix, tempJtW);
         CommonOps.multAdd(diagonalScalar, tempJtW, randomMatrix, expectedSolution);

         MatrixTools.multAddInner(diagonalScalar, randomMatrix, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, 1e-6);
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
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(taskSize, variables, -50.0, 50.0, random);
         DenseMatrix64F solution = RandomMatrices.createRandom(variables, variables, -50, 50, random);
         DenseMatrix64F expectedSolution = new DenseMatrix64F(solution);


         DenseMatrix64F tempJtW = new DenseMatrix64F(variables, taskSize);
         CommonOps.transpose(randomMatrix, tempJtW);
         CommonOps.multAdd(scale, tempJtW, randomMatrix, expectedSolution);

         MatrixTools.multAddInner(scale, randomMatrix, solution);

         JUnitTools.assertMatrixEquals(expectedSolution, solution, 1e-6);
      }
   }
}
