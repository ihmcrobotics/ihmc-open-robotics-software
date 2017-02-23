package us.ihmc.robotics.linearAlgebra;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class MatrixToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetMatrixColumnFromArrayDenseMatrix()
   {
      DenseMatrix64F test = new DenseMatrix64F(2, 2);

      double[] col = new double[] { 3.0, 4.0 };

      MatrixTools.setMatrixColumnFromArray(test, 1, col);

      assertEquals(col[0], test.get(0, 1), 1e-8);
      assertEquals(col[1], test.get(1, 1), 1e-8);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetMatrixFromOneBasedArrayDenseMatrix()
   {
      DenseMatrix64F test = new DenseMatrix64F(2, 1);

      double[] col = new double[] { 0.0, 3.0, 4.0 };

      MatrixTools.setMatrixFromOneBasedArray(test, col);

      assertEquals(col[1], test.get(0, 0), 1e-8);
      assertEquals(col[2], test.get(1, 0), 1e-8);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDiffDenseMatrixIntIntDenseMatrix()
   {
      double[][] vals = new double[][] { { 1.0 }, { 2.0 }, { 4.0 }, { 8.0 }, { 16.0 }, { 32.0 } };
      DenseMatrix64F test = new DenseMatrix64F(vals);

      DenseMatrix64F res = new DenseMatrix64F(2, 1);

      MatrixTools.diff(test, 2, 3, res);

      assertEquals(4.0, res.get(0, 0), 1e-8);
      assertEquals(8.0, res.get(1, 0), 1e-8);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDiffDoubleArrayDenseMatrix()
   {
      double[] vals = new double[] { 1.0, 3.0, 4.0, 9.0, 16.0, 32.0 };
      double[] expected = new double[] { 2.0, 1.0, 5.0, 7.0, 16.0 };
      DenseMatrix64F res = new DenseMatrix64F(5, 1);

      MatrixTools.diff(vals, res);

      for (int i = 0; i < 5; i++)
      {
         assertEquals(expected[i], res.get(i, 0), 1e-8);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRemoveRow()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 20; i++)
      {
         int numRows = RandomTools.generateRandomInt(random, 1, 100);
         int numCols = RandomTools.generateRandomInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         int indexOfRowToRemove = RandomTools.generateRandomInt(random, 0, randomMatrix.getNumRows() - 1);
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

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testRemoveZeroRows()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 200; i++)
      {
         int numRows = RandomTools.generateRandomInt(random, 1, 100);
         int numCols = RandomTools.generateRandomInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         int randomNumberOfZeroRows = RandomTools.generateRandomInt(random, 0, 5);
         int[] indicesOfZeroRows = RandomTools.generateRandomIntArray(random, randomNumberOfZeroRows, 0, randomMatrix.getNumRows() - 1);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testScaleTranspose() throws Exception
   {
      Random random = new Random(165156L);
      for (int i = 0; i < 200; i++)
      {
         int numRows = RandomTools.generateRandomInt(random, 1, 100);
         int numCols = RandomTools.generateRandomInt(random, 1, 100);
         DenseMatrix64F randomMatrix = RandomMatrices.createRandom(numRows, numCols, 1.0, 100.0, random);
         double randomAlpha = RandomTools.generateRandomDouble(random, 100.0);
         DenseMatrix64F expectedMatrix = new DenseMatrix64F(numCols, numRows);
         DenseMatrix64F actualMatrix = new DenseMatrix64F(numCols, numRows);

         CommonOps.transpose(randomMatrix, expectedMatrix);
         CommonOps.scale(randomAlpha, expectedMatrix);

         MatrixTools.scaleTranspose(randomAlpha, randomMatrix, actualMatrix);

         boolean areMatricesEqual = MatrixFeatures.isEquals(expectedMatrix, actualMatrix, 1.0e-10);
         assertTrue(areMatricesEqual);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInsertFrameTupleIntoEJMLVector()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 1000; i++)
      {
         int numRows = RandomTools.generateRandomInt(random, 3, 100);
         DenseMatrix64F matrixToTest = RandomMatrices.createRandom(numRows, 1, 1.0, 100.0, random);
         FramePoint framePointToInsert = FramePoint.generateRandomFramePoint(random, ReferenceFrame.getWorldFrame(), 100.0, 100.0, 100.0);
         int startRowToInsertFrameTuple = RandomTools.generateRandomInt(random, 0, numRows - 3);
         MatrixTools.insertFrameTupleIntoEJMLVector(framePointToInsert, matrixToTest, startRowToInsertFrameTuple);

         assertEquals(framePointToInsert.getX(), matrixToTest.get(startRowToInsertFrameTuple + 0, 0), 1.0e-10);
         assertEquals(framePointToInsert.getY(), matrixToTest.get(startRowToInsertFrameTuple + 1, 0), 1.0e-10);
         assertEquals(framePointToInsert.getZ(), matrixToTest.get(startRowToInsertFrameTuple + 2, 0), 1.0e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testExtractFrameTupleFromEJMLVector()
   {
      Random random = new Random(3216516L);
      for (int i = 0; i < 1000; i++)
      {
         int numRows = RandomTools.generateRandomInt(random, 3, 100);
         DenseMatrix64F matrixToExtractFrom = RandomMatrices.createRandom(numRows, 1, 1.0, 100.0, random);
         FramePoint framePointToTest = new FramePoint(null, -1.0, -1.0, -1.0);
         int startRowToExtractFrameTuple = RandomTools.generateRandomInt(random, 0, numRows - 3);
         MatrixTools.extractFrameTupleFromEJMLVector(framePointToTest, matrixToExtractFrom, ReferenceFrame.getWorldFrame(), startRowToExtractFrameTuple);

         assertEquals(framePointToTest.getReferenceFrame(), ReferenceFrame.getWorldFrame());
         assertEquals(framePointToTest.getX(), matrixToExtractFrom.get(startRowToExtractFrameTuple + 0, 0), 1.0e-10);
         assertEquals(framePointToTest.getY(), matrixToExtractFrom.get(startRowToExtractFrameTuple + 1, 0), 1.0e-10);
         assertEquals(framePointToTest.getZ(), matrixToExtractFrom.get(startRowToExtractFrameTuple + 2, 0), 1.0e-10);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
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
}
