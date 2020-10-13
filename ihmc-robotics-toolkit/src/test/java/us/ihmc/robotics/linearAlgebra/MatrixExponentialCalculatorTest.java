package us.ihmc.robotics.linearAlgebra;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
public class MatrixExponentialCalculatorTest
{

	@Test
   public void testAgainstTaylorSeries()
   {
      int size = 50;
      Random random = new Random(125L);
      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(size);

      DMatrixRMaj A = RandomMatrices_DDRM.rectangle(size, size, random);
      CommonOps_DDRM.scale(1.0 / size, A);


      DMatrixRMaj result = new DMatrixRMaj(size, size);
      matrixExponentialCalculator.compute(result, A);

      DMatrixRMaj taylorSeriesCheck = new DMatrixRMaj(size, size);
      computeMatrixExponentialThroughTaylorSeries(taylorSeriesCheck, A, 15);
      assertTrue(MatrixFeatures_DDRM.isIdentical(taylorSeriesCheck, result, 1e-6));
   }

	@Disabled
	@Test
   public void testEfficiency()
   {
      int size = 50;
      Random random = new Random(125L);
      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(size);

      DMatrixRMaj A = new DMatrixRMaj(size, size);
      DMatrixRMaj result = new DMatrixRMaj(size, size);

      int nTestsWarmup = 1000;
      for (int i = 0; i < nTestsWarmup; i++)
      {
         RandomMatrices_DDRM.fillUniform(A, random);
         CommonOps_DDRM.scale(5e-3, A); // time step
         matrixExponentialCalculator.compute(result, A);
      }
      
      int nTests = 10000;
      long totalTimeMillis = 0;
      for (int i = 0; i < nTests; i++)
      {
         RandomMatrices_DDRM.fillUniform(A, random);
         CommonOps_DDRM.scale(5e-3, A); // time step
         long startMillis = System.currentTimeMillis();
         matrixExponentialCalculator.compute(result, A);
         long stopMillis = System.currentTimeMillis();
         totalTimeMillis += stopMillis - startMillis;
      }
      double totalTimeMillisPerTest = ((double) totalTimeMillis) / nTests;
      System.out.println(totalTimeMillisPerTest);
   }
  
   private static void computeMatrixExponentialThroughTaylorSeries(DMatrixRMaj result, DMatrixRMaj A, int n)
   {
      DMatrixRMaj tempPower = new DMatrixRMaj(A.getNumRows(), A.getNumCols());
      DMatrixRMaj tempExponentialTerm = new DMatrixRMaj(A.getNumRows(), A.getNumCols());

      CommonOps_DDRM.setIdentity(result);

      CommonOps_DDRM.setIdentity(tempPower);
      int factorial = 1;
      for (int i = 1; i < n; i++)
      {
         factorial *= i;
         CommonOps_DDRM.mult(A, tempPower, tempExponentialTerm);
         tempPower.set(tempExponentialTerm);
         CommonOps_DDRM.scale(1.0 / factorial, tempExponentialTerm);
         CommonOps_DDRM.addEquals(result, tempExponentialTerm);
      }
   }
}
