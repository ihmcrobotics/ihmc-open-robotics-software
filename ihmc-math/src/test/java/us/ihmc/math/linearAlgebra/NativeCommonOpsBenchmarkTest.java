package us.ihmc.math.linearAlgebra;

import java.util.Random;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.dense.row.linsol.qr.LinearSolverQrHouseCol_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.matrixlib.NativeCommonOps;

public class NativeCommonOpsBenchmarkTest
{
   private static final int maxSize = 80;
   private static final int warmumIterations = 2000;
   private static final int iterations = 5000;
   private static final double epsilon = 1.0e-8;

   @Test
   public void testSolveLeastSquare()
   {
      Random random = new Random(40L);

      System.out.println("Testing solving least square problem with random matrices...");

      long nativeDampedTime = 0;
      long nativeUndampedTime = 0;
      long ejmlDampedTime = 0;
      long ejmlUndampedTime = 0;
      double matrixSizes = 0;
      double alpha = 0.01;
      int localMaxSize = (int) (maxSize * 4.0 / 3.0);
      LinearSolverDense<DMatrixRMaj> dampedSolver = new DampedLeastSquaresSolver(localMaxSize, alpha);
      LinearSolverDense<DMatrixRMaj> undampedSolver = new LinearSolverQrHouseCol_DDRM();

      for (int i = 0; i < warmumIterations; i++)
      {
         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(maxSize, maxSize, random);
         DMatrixRMaj x = RandomMatrices_DDRM.rectangle(maxSize, 1, random);
         DMatrixRMaj b = new DMatrixRMaj(maxSize, 1);
         CommonOps_DDRM.mult(A, x, b);
         dampedSolver.setA(A);
         dampedSolver.solve(b, x);
         undampedSolver.setA(A);
         undampedSolver.solve(b, x);
         NativeCommonOps.solveDamped(A, b, alpha, x);
         NativeCommonOps.solveRobust(A, b, x);
      }

      for (int i = 0; i < iterations; i++)
      {
         int aRows = random.nextInt(localMaxSize) + 2;
         int aCols = random.nextInt(aRows - 1) + 1;
         matrixSizes += (aRows + aCols) / 2.0;

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(aRows, aCols, random);
         DMatrixRMaj x = RandomMatrices_DDRM.rectangle(aCols, 1, random);
         DMatrixRMaj b = new DMatrixRMaj(aRows, 1);
         CommonOps_DDRM.mult(A, x, b);

         DMatrixRMaj nativeDampedResult = new DMatrixRMaj(aCols, 1);
         DMatrixRMaj nativeUndampedResult = new DMatrixRMaj(aCols, 1);
         DMatrixRMaj ejmlDampedResult = new DMatrixRMaj(aCols, 1);
         DMatrixRMaj ejmlUndampedResult = new DMatrixRMaj(aCols, 1);

         nativeDampedTime -= System.nanoTime();
         NativeCommonOps.solveDamped(A, b, alpha, nativeDampedResult);
         nativeDampedTime += System.nanoTime();

         nativeUndampedTime -= System.nanoTime();
         NativeCommonOps.solveRobust(A, b, nativeUndampedResult);
         nativeUndampedTime += System.nanoTime();

         ejmlDampedTime -= System.nanoTime();
         dampedSolver.setA(A);
         dampedSolver.solve(b, ejmlDampedResult);
         ejmlDampedTime += System.nanoTime();

         ejmlUndampedTime -= System.nanoTime();
         undampedSolver.setA(A);
         undampedSolver.solve(b, ejmlUndampedResult);
         ejmlUndampedTime += System.nanoTime();

         EjmlUnitTests.assertEquals(ejmlDampedResult, nativeDampedResult, epsilon);
         EjmlUnitTests.assertEquals(x, nativeUndampedResult, epsilon);
         EjmlUnitTests.assertEquals(x, ejmlUndampedResult, epsilon);
      }

      System.out.println("Native damped took " + Conversions.nanosecondsToMilliseconds((double) (nativeDampedTime / iterations)) + " ms on average");
      System.out.println("Native undamped took " + Conversions.nanosecondsToMilliseconds((double) (nativeUndampedTime / iterations)) + " ms on average");
      System.out.println("EJML damped took " + Conversions.nanosecondsToMilliseconds((double) (ejmlDampedTime / iterations)) + " ms on average");
      System.out.println("EJML undamped took " + Conversions.nanosecondsToMilliseconds((double) (ejmlUndampedTime / iterations)) + " ms on average");
      System.out.println("Average matrix size was " + matrixSizes / iterations);
      System.out.println("Native damped takes " + (100.0 * nativeDampedTime / ejmlDampedTime) + "% of EJML time.");
      System.out.println("Native undamped takes " + (100.0 * nativeUndampedTime / ejmlUndampedTime) + "% of EJML time.\n");
   }

   @Test
   public void testNullspaceProjection()
   {
      Random random = new Random(40L);

      System.out.println("Testing nullspace projection with random matrices...");

      long nativeTime = 0;
      long ejmlTime = 0;
      double matrixSizes = 0.0;
      double alpha = 0.05;

      DampedLeastSquaresNullspaceCalculator calculator = new DampedLeastSquaresNullspaceCalculator(maxSize, alpha);

      for (int i = 0; i < warmumIterations; i++)
      {
         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(maxSize, maxSize, random);
         DMatrixRMaj B = RandomMatrices_DDRM.rectangle(maxSize, maxSize, random);
         DMatrixRMaj C = new DMatrixRMaj(maxSize, maxSize);
         calculator.projectOntoNullspace(A, B, C);
         NativeCommonOps.projectOnNullspace(A, B, C, alpha);
      }

      for (int i = 0; i < iterations; i++)
      {
         int aRows = random.nextInt(maxSize) + 1;
         int bRows = random.nextInt(maxSize) + 1;
         int bCols = random.nextInt(maxSize) + 1;
         matrixSizes += (aRows + bRows + bCols) / 3.0;

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(aRows, bCols, random);
         DMatrixRMaj B = RandomMatrices_DDRM.rectangle(bRows, bCols, random);
         DMatrixRMaj actual = new DMatrixRMaj(aRows, bCols);
         DMatrixRMaj expected = new DMatrixRMaj(aRows, bCols);

         nativeTime -= System.nanoTime();
         NativeCommonOps.projectOnNullspace(A, B, actual, alpha);
         nativeTime += System.nanoTime();

         ejmlTime -= System.nanoTime();
         calculator.projectOntoNullspace(A, B, expected);
         ejmlTime += System.nanoTime();

         EjmlUnitTests.assertEquals(expected, actual, epsilon);
      }

      System.out.println("Native took " + Conversions.nanosecondsToMilliseconds((double) (nativeTime / iterations)) + " ms on average");
      System.out.println("EJML took " + Conversions.nanosecondsToMilliseconds((double) (ejmlTime / iterations)) + " ms on average");
      System.out.println("Average matrix size was " + matrixSizes / iterations);
      System.out.println("Native takes " + (100.0 * nativeTime / ejmlTime) + "% of EJML time.\n");
   }

   public static void main(String[] args)
   {
      int size = 500;
      Random random = new Random(40L);
      DMatrixRMaj A = RandomMatrices_DDRM.rectangle(size, size, random);
      DMatrixRMaj B = RandomMatrices_DDRM.rectangle(size, size, random);
      DMatrixRMaj AtBA = new DMatrixRMaj(size, size);

      System.out.println("Running...");

      while (true)
      {
         NativeCommonOps.multQuad(A, B, AtBA);
      }
   }
}
