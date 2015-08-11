package us.ihmc.robotics.linearAlgebra;

import Jama.Matrix;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Point3d;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class MatrixToolsTest
{

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testCompare()
   {
      Matrix A = Matrix.random(6, 6);
      Matrix B = Matrix.random(6, 6);
      Matrix C = new Matrix(5, 6);
      Matrix D = new Matrix(6, 6);

      Matrix Acopy = new Matrix(A.getArray());

      assertFalse(MatrixTools.compare(A, B, 1e-8));
      assertFalse(MatrixTools.compare(C, D, 1e-8));
      assertTrue(MatrixTools.compare(A, B, 2.0));

      assertTrue(MatrixTools.compare(A, Acopy, 1e-8));

      assertFalse(MatrixTools.compare(C, D, 1e-8));

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN()
   {
      Matrix test = new Matrix(3, 3);
      MatrixTools.setToNaN(test);

      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            assertTrue(Double.isNaN(test.get(i, j)));
         }
      }
   }

   @EstimatedDuration(duration = 0.0)
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

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testSetToZero()
   {
      Matrix test = new Matrix(3, 3);
      MatrixTools.setToZero(test);

      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            assertEquals(0.0, test.get(i, j), 1e-34);
         }
      }
   }

   @EstimatedDuration(duration = 0.0)
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

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testSetMatrixColumnFromArray()
   {
      Matrix test = new Matrix(2, 2);

      double[] col = new double[] { 3.0, 4.0 };

      MatrixTools.setMatrixColumnFromArray(test, 1, col);

      assertEquals(col[0], test.get(0, 1), 1e-8);
      assertEquals(col[1], test.get(1, 1), 1e-8);

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testSetMatrixColumnFromArrayDenseMatrix()
   {
      DenseMatrix64F test = new DenseMatrix64F(2, 2);

      double[] col = new double[] { 3.0, 4.0 };

      MatrixTools.setMatrixColumnFromArray(test, 1, col);

      assertEquals(col[0], test.get(0, 1), 1e-8);
      assertEquals(col[1], test.get(1, 1), 1e-8);

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testSetMatrixFromOneBasedArray()
   {
      Matrix test = new Matrix(2, 1);

      double[] col = new double[] { 0.0, 3.0, 4.0 };

      MatrixTools.setMatrixFromOneBasedArray(test, col);

      assertEquals(col[1], test.get(0, 0), 1e-8);
      assertEquals(col[2], test.get(1, 0), 1e-8);

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testSetMatrixFromOneBasedArrayDenseMatrix()
   {
      DenseMatrix64F test = new DenseMatrix64F(2, 1);

      double[] col = new double[] { 0.0, 3.0, 4.0 };

      MatrixTools.setMatrixFromOneBasedArray(test, col);

      assertEquals(col[1], test.get(0, 0), 1e-8);
      assertEquals(col[2], test.get(1, 0), 1e-8);

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testDiffMatrixIntIntMatrix()
   {
      double[][] vals = new double[][] { { 1.0 }, { 2.0 }, { 4.0 }, { 8.0 }, { 16.0 }, { 32.0 } };
      Matrix test = new Matrix(vals);

      Matrix res = new Matrix(2, 1);

      MatrixTools.diff(test, 2, 3, res);

      assertEquals(4.0, res.get(0, 0), 1e-8);
      assertEquals(8.0, res.get(1, 0), 1e-8);

   }

   @EstimatedDuration(duration = 0.0)
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

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testDiffMatrixMatrix()
   {
      double[][] vals = new double[][] { { 1.0 }, { 3.0 }, { 4.0 }, { 9.0 }, { 16.0 }, { 32.0 } };
      double[] expected = new double[] { 2.0, 1.0, 5.0, 7.0, 16.0 };
      Matrix test = new Matrix(vals);

      Matrix res = new Matrix(5, 1);

      MatrixTools.diff(test, res);

      for (int i = 0; i < 5; i++)
      {
         assertEquals(expected[i], res.get(i, 0), 1e-8);
      }

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testDiffDoubleArrayMatrix()
   {
      double[] vals = new double[] { 1.0, 3.0, 4.0, 9.0, 16.0, 32.0 };
      double[] expected = new double[] { 2.0, 1.0, 5.0, 7.0, 16.0 };
      Matrix res = new Matrix(5, 1);

      MatrixTools.diff(vals, res);

      for (int i = 0; i < 5; i++)
      {
         assertEquals(expected[i], res.get(i, 0), 1e-8);
      }
   }

   @EstimatedDuration(duration = 0.0)
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

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testSetMatrixBlock()
   {
      Matrix test = Matrix.random(6, 6);
      Matrix test2 = Matrix.random(6, 6);

      MatrixTools.setMatrixBlock(test, 2, 1, test2, 0, 1, 2, 3, 2.0);

      Matrix resA = test.getMatrix(2, 3, 1, 3);
      Matrix resB = test2.getMatrix(0, 1, 1, 3);
      resB.timesEquals(2.0);

      assertTrue(MatrixTools.compare(resA, resB, 1e-8));
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testAddMatrixBlock()
   {
      Matrix test = Matrix.random(6, 6);
      Matrix test2 = Matrix.random(6, 6);

      Matrix resA = test.getMatrix(2, 3, 1, 3);
      MatrixTools.addMatrixBlock(test, 2, 1, test2, 0, 1, 2, 3, 2.0);

      Matrix resB = test.getMatrix(2, 3, 1, 3);

      Matrix resC = test2.getMatrix(0, 1, 1, 3);
      resC.timesEquals(2.0);

      resB.minusEquals(resC);

      assertTrue(MatrixTools.compare(resA, resB, 1e-8));
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testTimes()
   {
      Matrix test = Matrix.random(6, 6);
      Matrix test2 = Matrix.random(6, 6);

      Matrix resA = test.times(test2);
      Matrix resB = new Matrix(6, 6);

      MatrixTools.times(test, test2, resB);

      assertTrue(MatrixTools.compare(resA, resB, 1e-8));

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void tranformSe3IntoTransform3D()
   {
      Se3_F64 a = new Se3_F64();
      RotationMatrixGenerator.eulerXYZ(0.1, -0.5, 1.2, a.getR());
      a.getT().set(3.3, 1.2, -9);

      RigidBodyTransform b = new RigidBodyTransform();
      MatrixTools.tranformSe3IntoTransform3D(a, b);

      Point3D_F64 p0 = new Point3D_F64(-1, 2, 3);
      Point3d p1 = new Point3d(p0.x, p0.y, p0.z);

      SePointOps_F64.transform(a, p0, p0);

      b.transform(p1);

      assertEquals(p0.x, p1.x, 1e-8);
      assertEquals(p0.y, p1.y, 1e-8);
      assertEquals(p0.z, p1.z, 1e-8);
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testPseudoinverse()
   {
      // compare against MATLAB pinv function

      double[][] Md =
      {
         {
            0.7060, 0.0462, 0.6948, 0.0344, 0.7655, 0.4898
         },
         {
            0.0318, 0.0971, 0.3171, 0.4387, 0.7952, 0.4456
         },
         {
            0.2769, 0.8235, 0.9502, 0.3816, 0.1869, 0.6463
         }
      };

      Matrix M = new Matrix(Md);
      Matrix M_inv = MatrixTools.pseudoinverse(M);

      double[][] expectedReturn_M_inv =
      {
         {0.936998438717307, -0.785129603103276, -0.098850772280042}, {-0.415529615479199, -0.090901192323565, 0.656784574946038},
         {0.296554021230810, -0.353448405794565, 0.397328610544366}, {-0.601416043928672, 0.809418197290933, 0.176759435487931},
         {0.239240144723877, 0.856358371010319, -0.439451934635555}, {-0.022171162226724, 0.246409897051293, 0.191341831239165}
      };

      assertEquals(M_inv.getRowDimension(), 6);
      assertEquals(M_inv.getColumnDimension(), 3);

      // can't use Double.MIN_VALUE here, so I'm making something reasonable up
      double epsilon = 1e-3;

      for (int row = 0; row < M_inv.getRowDimension(); row++)
      {
         for (int col = 0; col < M_inv.getColumnDimension(); col++)
         {
            assertEquals(M_inv.get(row, col), expectedReturn_M_inv[row][col], epsilon);
         }
      }

      // can't use Double.MIN_VALUE here, so I'm making something reasonable up
      double epsilon2 = 1e-9;

      double[][] Ad =
      {
         {0.243524968724989, 0.196595250431208, 0.473288848902729}, {0.929263623187228, 0.251083857976031, 0.351659507062997},
         {0.349983765984809, 0.616044676146639, 0.830828627896291}
      };

      Matrix A = new Matrix(Ad);
      Matrix A_inv = MatrixTools.pseudoinverse(A);

      assertEquals(A_inv.getRowDimension(), 3);
      assertEquals(A_inv.getColumnDimension(), 3);

      Matrix eye = A.times(A_inv);

      for (int row = 0; row < eye.getRowDimension(); row++)
      {
         for (int col = 0; col < eye.getColumnDimension(); col++)
         {
            if (row == col)
            {
               assertEquals(eye.get(row, col), 1.0, epsilon2);
            }
            else
            {
               assertEquals(eye.get(row, col), 0.0, epsilon2);
            }
         }
      }

   }

   @EstimatedDuration(duration = 0.02)
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

   @EstimatedDuration(duration = 0.02)
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

   @EstimatedDuration(duration = 0.0)
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

   @EstimatedDuration(duration = 0.0)
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
   
   @EstimatedDuration(duration = 0.4)
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
