package us.ihmc.robotics.linearAlgebra;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

public class MatrixExponentialCalculatorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAgainstTaylorSeries()
   {
      int size = 50;
      Random random = new Random(125L);
      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(size);

      DenseMatrix64F A = RandomMatrices.createRandom(size, size, random);
      CommonOps.scale(1.0 / size, A);


      DenseMatrix64F result = new DenseMatrix64F(size, size);
      matrixExponentialCalculator.compute(result, A);

      DenseMatrix64F taylorSeriesCheck = new DenseMatrix64F(size, size);
      computeMatrixExponentialThroughTaylorSeries(taylorSeriesCheck, A, 15);
      assertTrue(MatrixFeatures.isIdentical(taylorSeriesCheck, result, 1e-6));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testEfficiency()
   {
      int size = 50;
      Random random = new Random(125L);
      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(size);

      DenseMatrix64F A = new DenseMatrix64F(size, size);
      DenseMatrix64F result = new DenseMatrix64F(size, size);

      int nTestsWarmup = 1000;
      for (int i = 0; i < nTestsWarmup; i++)
      {
         RandomMatrices.setRandom(A, random);
         CommonOps.scale(5e-3, A); // time step
         matrixExponentialCalculator.compute(result, A);
      }
      
      int nTests = 10000;
      long totalTimeMillis = 0;
      for (int i = 0; i < nTests; i++)
      {
         RandomMatrices.setRandom(A, random);
         CommonOps.scale(5e-3, A); // time step
         long startMillis = System.currentTimeMillis();
         matrixExponentialCalculator.compute(result, A);
         long stopMillis = System.currentTimeMillis();
         totalTimeMillis += stopMillis - startMillis;
      }
      double totalTimeMillisPerTest = ((double) totalTimeMillis) / nTests;
      System.out.println(totalTimeMillisPerTest);
   }
  
   private static void computeMatrixExponentialThroughTaylorSeries(DenseMatrix64F result, DenseMatrix64F A, int n)
   {
      DenseMatrix64F tempPower = new DenseMatrix64F(A.getNumRows(), A.getNumCols());
      DenseMatrix64F tempExponentialTerm = new DenseMatrix64F(A.getNumRows(), A.getNumCols());

      CommonOps.setIdentity(result);

      CommonOps.setIdentity(tempPower);
      int factorial = 1;
      for (int i = 1; i < n; i++)
      {
         factorial *= i;
         CommonOps.mult(A, tempPower, tempExponentialTerm);
         tempPower.set(tempExponentialTerm);
         CommonOps.scale(1.0 / factorial, tempExponentialTerm);
         CommonOps.addEquals(result, tempExponentialTerm);
      }
   }
}
