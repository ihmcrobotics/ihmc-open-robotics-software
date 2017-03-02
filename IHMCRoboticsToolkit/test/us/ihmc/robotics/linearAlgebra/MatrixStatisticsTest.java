package us.ihmc.robotics.linearAlgebra;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import Jama.Matrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.MathTools;

public class MatrixStatisticsTest
{
   private static final int ITERATIONS = 100;
   private static final double MAXDOUBLE = 1e5;
   private static final int MAXROWS = 15;
   private static final int MAXCOLUMNS = 15;
   private static final double DELTA = 1e-3;
   private Random random;

   @Before
   public void setUp()
   {
      random = new Random();
   }

   public int randomDimension(int maximum)
   {
      if (maximum == 1)
         return 1;

      return random.nextInt(maximum - 1) + 1;
   }

   public double randomDouble()
   {
      return random.nextDouble() * MAXDOUBLE * 2.0 - MAXDOUBLE;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIndecesOfMaxElement()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);
         int columns = randomDimension(MAXCOLUMNS);
         Matrix matrix = new Matrix(rows, columns);

         int[] indices = MatrixStatistics.indecesOfMaxElement(matrix);
         assertEquals(0, indices[0]);
         assertEquals(0, indices[1]);

         int rowMaxValue = randomDimension(rows) - 1;
         int columnMaxValue = randomDimension(columns) - 1;
         double maxValue = Math.abs(randomDouble());
         matrix.set(rowMaxValue, columnMaxValue, maxValue);

         indices = MatrixStatistics.indecesOfMaxElement(matrix);
         assertEquals(rowMaxValue, indices[0]);
         assertEquals(columnMaxValue, indices[1]);
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetCovarianceMatrix()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);
         int columns = randomDimension(MAXCOLUMNS);

         Matrix matrix = new Matrix(rows, columns, 0.0);

         double[] averageColumn = new double[rows];
         for (int m = 0; m < rows; m++)
         {
            averageColumn[m] = 0.0;

            for (int n = 0; n < columns; n++)
            {
               double value = randomDouble();
               averageColumn[m] += value / ((double) columns);
               matrix.set(m, n, value);
            }
         }
         
         Matrix covarianceTest = new Matrix(rows, rows, 0.0);
         for(int m = 0; m < rows; m++)
         {
        	 for( int n = 0; n < rows; n++)
        	 {
        		 double covariance = 0.0;
        		 for(int o = 0; o < columns; o++)
        		 {
        			 covariance += (matrix.get(m, o) - averageColumn[m]) * (matrix.get(n, o) - averageColumn[n]) / ((double)columns);
        		 }
        		 covarianceTest.set(m, n, covariance);
        	 }
         }
                  
         Matrix covarianceMatrix = MatrixStatistics.getCovarianceMatrix(matrix);

         assertEquals(covarianceTest.getRowDimension(), covarianceMatrix.getRowDimension());
         assertEquals(covarianceTest.getColumnDimension(), covarianceMatrix.getColumnDimension());
         
         for(int m = 0; m < rows; m++)
         {
        	 for( int n = 0; n < rows; n++)
        	 {
        		 assertEquals(covarianceMatrix.get(m,n), covarianceMatrix.get(n,m), DELTA); // Symmetry
        		 assertEquals(covarianceTest.get(m, n), covarianceMatrix.get(m, n), DELTA); // Value
        	 }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSubtractAverageColumnFromEachRow()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);
         int columns = randomDimension(MAXCOLUMNS);

         Matrix matrix = new Matrix(rows, columns, 0.0);

         double[] averages = new double[rows];
         for (int m = 0; m < rows; m++)
         {
            averages[m] = 0.0;

            for (int n = 0; n < columns; n++)
            {
               double value = randomDouble();
               averages[m] += value / ((double) columns);
               matrix.set(m, n, value);
            }
         }

         Matrix subtractedAverageMatrix = MatrixStatistics.subtractAverageColumnFromEachRow(matrix);

         for (int m = 0; m < rows; m++)
         {
            for (int n = 0; n < columns; n++)
            {
               assertEquals(matrix.get(m, n) - averages[m], subtractedAverageMatrix.get(m, n), DELTA);
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSumAllElements()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);
         int columns = randomDimension(MAXCOLUMNS);

         double sum = 0.0;
         Matrix matrix = new Matrix(rows, columns, 0.0);

         for (int m = 0; m < rows; m++)
         {
            for (int n = 0; n < columns; n++)
            {
               double value = randomDouble();
               sum += value;
               matrix.set(m, n, value);
            }
         }

         assertEquals(sum, MatrixStatistics.sumAllElements(matrix), DELTA);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDivideEachRowByStdDevOfRow()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);
         int columns = randomDimension(MAXCOLUMNS);

         Matrix matrix = new Matrix(rows, columns, 0.0);

         for (int m = 0; m < rows; m++)
         {
            for (int n = 0; n < columns; n++)
            {
               double value = randomDouble();
               matrix.set(m, n, value);
            }
         }

         Matrix variances = MatrixStatistics.getVarianceOfEachRow(matrix);
         Matrix dividedByStdDev = MatrixStatistics.divideEachRowByStdDevOfRow(matrix);

         for (int m = 0; m < rows; m++)
         {
            for (int n = 0; n < columns; n++)
            {
               assertEquals(matrix.get(m, n) / Math.sqrt(variances.get(m, 0)), dividedByStdDev.get(m, n), DELTA);
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetVarianceOfEachRow()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);
         int columns = randomDimension(MAXCOLUMNS);

         Matrix matrix = new Matrix(rows, columns, 0.0);

         double[] rowSums = new double[rows];
         for (int m = 0; m < rows; m++)
         {
            rowSums[m] = 0.0;

            for (int n = 0; n < columns; n++)
            {
               double value = randomDouble();
               rowSums[m] += value;
               matrix.set(m, n, value);
            }
         }

         Matrix variancesMatrix = MatrixStatistics.getVarianceOfEachRow(matrix);

         double[][] internalMatrix = matrix.getArrayCopy();
         for (int m = 0; m < matrix.getRowDimension(); m++)
         {
            double[] row = internalMatrix[m];
            double average = rowSums[m] / ((double) columns);
            double variance = 0.0;

            for (int n = 0; n < row.length; n++)
            {
               variance += MathTools.square(row[n] - average) / ((double) columns);
            }

            assertEquals(variance, variancesMatrix.get(m, 0), DELTA);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetAverageColumnVector()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);
         int columns = randomDimension(MAXCOLUMNS);

         Matrix matrix = new Matrix(rows, columns, 0.0);

         double[] rowAverages = new double[rows];
         for (int m = 0; m < rows; m++)
         {
            rowAverages[m] = 0.0;

            for (int n = 0; n < columns; n++)
            {
               double value = randomDouble();
               rowAverages[m] += value / ((double) columns);
               matrix.set(m, n, value);
            }
         }

         Matrix averageColumnVector = MatrixStatistics.getAverageColumnVector(matrix);
         for (int m = 0; m < rows; m++)
         {
            assertEquals(rowAverages[m], averageColumnVector.get(m, 0), DELTA);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateColumnVectorIntDouble()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);
         double value = randomDouble();

         Matrix matrix = MatrixStatistics.createColumnVector(rows, value);

         assertEquals(rows, matrix.getRowDimension());
         assertEquals(1, matrix.getColumnDimension());

         for (int m = 0; m < rows; m++)
         {
            assertEquals(value, matrix.get(m, 0), DELTA);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateRowVectorIntDouble()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int columns = randomDimension(MAXCOLUMNS);
         double value = randomDouble();

         Matrix matrix = MatrixStatistics.createRowVector(columns, value);

         assertEquals(1, matrix.getRowDimension());
         assertEquals(columns, matrix.getColumnDimension());

         for (int n = 0; n < columns; n++)
         {
            assertEquals(value, matrix.get(0, n), DELTA);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateColumnVectorInt()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int rows = randomDimension(MAXROWS);

         Matrix matrix = MatrixStatistics.createColumnVector(rows);

         assertEquals(rows, matrix.getRowDimension());
         assertEquals(1, matrix.getColumnDimension());

         for (int m = 0; m < rows; m++)
         {
            assertEquals(0.0, matrix.get(m, 0), DELTA);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateRowVectorInt()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int columns = randomDimension(MAXCOLUMNS);

         Matrix matrix = MatrixStatistics.createRowVector(columns);

         assertEquals(1, matrix.getRowDimension());
         assertEquals(columns, matrix.getColumnDimension());

         for (int n = 0; n < columns; n++)
         {
            assertEquals(0.0, matrix.get(0, n), DELTA);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testGetRowNumber()
   {
      fail("Not yet implemented");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testGetColumnNumber()
   {
      fail("Not yet implemented");
   }

}
