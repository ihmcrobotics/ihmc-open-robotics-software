package us.ihmc.robotics.linearAlgebra.commonOps;

import java.util.Random;

import org.apache.commons.math3.util.Precision;
import org.ejml.alg.dense.linsol.qr.LinearSolverQrHouseCol_D64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Conversions;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.DampedLeastSquaresNullspaceCalculator;
import us.ihmc.robotics.testing.JUnitTools;

public class NativeCommonOpsTest
{
   private static final int maxSize = 80;
   private static final int warmumIterations = 2000;
   private static final int iterations = 5000;
   private static final double epsilon = 1.0e-8;

   @Test
   public void testMult()
   {
      Random random = new Random(40L);

      System.out.println("Testing matrix multiplications with random matrices...");

      long nativeTime = 0;
      long ejmlTime = 0;
      double matrixSizes = 0.0;

      for (int i = 0; i < warmumIterations; i++)
      {
         DenseMatrix64F A = RandomMatrices.createRandom(maxSize, maxSize, random);
         DenseMatrix64F B = RandomMatrices.createRandom(maxSize, maxSize, random);
         DenseMatrix64F AB = new DenseMatrix64F(maxSize, maxSize);
         CommonOps.mult(A, B, AB);
         NativeCommonOps.mult(A, B, AB);
      }

      for (int i = 0; i < iterations; i++)
      {
         int aRows = random.nextInt(maxSize) + 1;
         int aCols = random.nextInt(maxSize) + 1;
         int bCols = random.nextInt(maxSize) + 1;
         matrixSizes += (aRows + aCols + bCols) / 3.0;

         DenseMatrix64F A = RandomMatrices.createRandom(aRows, aCols, random);
         DenseMatrix64F B = RandomMatrices.createRandom(aCols, bCols, random);
         DenseMatrix64F actual = new DenseMatrix64F(aRows, bCols);
         DenseMatrix64F expected = new DenseMatrix64F(aRows, bCols);

         nativeTime -= System.nanoTime();
         NativeCommonOps.mult(A, B, actual);
         nativeTime += System.nanoTime();

         ejmlTime -= System.nanoTime();
         CommonOps.mult(A, B, expected);
         ejmlTime += System.nanoTime();

         JUnitTools.assertMatrixEquals(expected, actual, epsilon);
      }

      System.out.println("Native took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (nativeTime / iterations)), 3) + " ms on average");
      System.out.println("EJML took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (ejmlTime / iterations)), 3) + " ms on average");
      System.out.println("Average matrix size was " + Precision.round(matrixSizes / iterations, 1));
      System.out.println("Native takes " + Precision.round((100.0 * nativeTime / ejmlTime), 0) + "% of EJML time.\n");
   }

   @Test
   public void testMultQuad()
   {
      Random random = new Random(40L);

      System.out.println("Testing computing quadratic form with random matrices...");

      long nativeTime = 0;
      long ejmlTime = 0;
      double matrixSizes = 0.0;

      for (int i = 0; i < warmumIterations; i++)
      {
         DenseMatrix64F A = RandomMatrices.createRandom(maxSize, maxSize, random);
         DenseMatrix64F B = RandomMatrices.createRandom(maxSize, maxSize, random);
         DenseMatrix64F tempBA = new DenseMatrix64F(maxSize, maxSize);
         DenseMatrix64F AtBA = new DenseMatrix64F(maxSize, maxSize);
         CommonOps.mult(B, A, tempBA);
         CommonOps.multTransA(A, tempBA, AtBA);
         NativeCommonOps.multQuad(A, B, AtBA);
      }

      for (int i = 0; i < iterations; i++)
      {
         int aRows = random.nextInt(maxSize) + 1;
         int aCols = random.nextInt(maxSize) + 1;
         matrixSizes += (aRows + aCols) / 2.0;

         DenseMatrix64F A = RandomMatrices.createRandom(aRows, aCols, random);
         DenseMatrix64F B = RandomMatrices.createRandom(aRows, aRows, random);
         DenseMatrix64F actual = new DenseMatrix64F(aCols, aCols);
         DenseMatrix64F expected = new DenseMatrix64F(aCols, aCols);
         DenseMatrix64F tempBA = new DenseMatrix64F(aRows, aCols);

         nativeTime -= System.nanoTime();
         NativeCommonOps.multQuad(A, B, actual);
         nativeTime += System.nanoTime();

         ejmlTime -= System.nanoTime();
         CommonOps.mult(B, A, tempBA);
         CommonOps.multTransA(A, tempBA, expected);
         ejmlTime += System.nanoTime();

         JUnitTools.assertMatrixEquals(expected, actual, epsilon);
      }

      System.out.println("Native took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (nativeTime / iterations)), 3) + " ms on average");
      System.out.println("EJML took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (ejmlTime / iterations)), 3) + " ms on average");
      System.out.println("Average matrix size was " + Precision.round(matrixSizes / iterations, 1));
      System.out.println("Native takes " + Precision.round((100.0 * nativeTime / ejmlTime), 0) + "% of EJML time.\n");
   }

   @Test
   public void testInvert()
   {
      Random random = new Random(40L);

      System.out.println("Testing inverting with random matrices...");

      long nativeTime = 0;
      long ejmlTime = 0;
      double matrixSizes = 0;
      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.lu(maxSize);

      for (int i = 0; i < warmumIterations; i++)
      {
         DenseMatrix64F A = RandomMatrices.createRandom(maxSize, maxSize, -100.0, 100.0, random);
         DenseMatrix64F B = new DenseMatrix64F(maxSize, maxSize);
         solver.setA(A);
         solver.invert(B);
         NativeCommonOps.invert(A, B);
      }

      for (int i = 0; i < iterations; i++)
      {
         int aRows = random.nextInt(maxSize) + 1;
         matrixSizes += aRows;

         DenseMatrix64F A = RandomMatrices.createRandom(aRows, aRows, -100.0, 100.0, random);
         DenseMatrix64F nativeResult = new DenseMatrix64F(aRows, aRows);
         DenseMatrix64F ejmlResult = new DenseMatrix64F(aRows, aRows);

         nativeTime -= System.nanoTime();
         NativeCommonOps.invert(A, nativeResult);
         nativeTime += System.nanoTime();

         ejmlTime -= System.nanoTime();
         solver.setA(A);
         solver.invert(ejmlResult);
         ejmlTime += System.nanoTime();

         JUnitTools.assertMatrixEquals(ejmlResult, nativeResult, epsilon);
      }

      System.out.println("Native took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (nativeTime / iterations)), 3) + " ms on average");
      System.out.println("EJML took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (ejmlTime / iterations)), 3) + " ms on average");
      System.out.println("Average matrix size was " + Precision.round(matrixSizes / iterations, 1));
      System.out.println("Native takes " + Precision.round((100.0 * nativeTime / ejmlTime), 0) + "% of EJML time.\n");
   }

   @Test
   public void testSolve()
   {
      Random random = new Random(40L);

      System.out.println("Testing solving linear equations with random matrices...");

      long nativeTime = 0;
      long ejmlTime = 0;
      double matrixSizes = 0;
      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.lu(maxSize);

      for (int i = 0; i < warmumIterations; i++)
      {
         DenseMatrix64F A = RandomMatrices.createRandom(maxSize, maxSize, random);
         DenseMatrix64F x = RandomMatrices.createRandom(maxSize, 1, random);
         DenseMatrix64F b = new DenseMatrix64F(maxSize, 1);
         CommonOps.mult(A, x, b);
         solver.setA(A);
         solver.solve(b, x);
         NativeCommonOps.solve(A, b, x);
      }

      for (int i = 0; i < iterations; i++)
      {
         int aRows = random.nextInt(maxSize) + 1;
         matrixSizes += aRows;

         DenseMatrix64F A = RandomMatrices.createRandom(aRows, aRows, random);
         DenseMatrix64F x = RandomMatrices.createRandom(aRows, 1, random);
         DenseMatrix64F b = new DenseMatrix64F(aRows, 1);
         CommonOps.mult(A, x, b);

         DenseMatrix64F nativeResult = new DenseMatrix64F(aRows, 1);
         DenseMatrix64F ejmlResult = new DenseMatrix64F(aRows, 1);

         nativeTime -= System.nanoTime();
         NativeCommonOps.solve(A, b, nativeResult);
         nativeTime += System.nanoTime();

         ejmlTime -= System.nanoTime();
         solver.setA(A);
         solver.solve(b, ejmlResult);
         ejmlTime += System.nanoTime();

         JUnitTools.assertMatrixEquals(x, nativeResult, epsilon);
         JUnitTools.assertMatrixEquals(x, ejmlResult, epsilon);
      }

      System.out.println("Native took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (nativeTime / iterations)), 3) + " ms on average");
      System.out.println("EJML took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (ejmlTime / iterations)), 3) + " ms on average");
      System.out.println("Average matrix size was " + Precision.round(matrixSizes / iterations, 1));
      System.out.println("Native takes " + Precision.round((100.0 * nativeTime / ejmlTime), 0) + "% of EJML time.\n");
   }

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
      LinearSolver<DenseMatrix64F> dampedSolver = new DampedLeastSquaresSolver(localMaxSize, alpha);
      LinearSolver<DenseMatrix64F> undampedSolver = new LinearSolverQrHouseCol_D64();

      for (int i = 0; i < warmumIterations; i++)
      {
         DenseMatrix64F A = RandomMatrices.createRandom(maxSize, maxSize, random);
         DenseMatrix64F x = RandomMatrices.createRandom(maxSize, 1, random);
         DenseMatrix64F b = new DenseMatrix64F(maxSize, 1);
         CommonOps.mult(A, x, b);
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

         DenseMatrix64F A = RandomMatrices.createRandom(aRows, aCols, random);
         DenseMatrix64F x = RandomMatrices.createRandom(aCols, 1, random);
         DenseMatrix64F b = new DenseMatrix64F(aRows, 1);
         CommonOps.mult(A, x, b);

         DenseMatrix64F nativeDampedResult = new DenseMatrix64F(aCols, 1);
         DenseMatrix64F nativeUndampedResult = new DenseMatrix64F(aCols, 1);
         DenseMatrix64F ejmlDampedResult = new DenseMatrix64F(aCols, 1);
         DenseMatrix64F ejmlUndampedResult = new DenseMatrix64F(aCols, 1);

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

         JUnitTools.assertMatrixEquals(ejmlDampedResult, nativeDampedResult, epsilon);
         JUnitTools.assertMatrixEquals(x, nativeUndampedResult, epsilon);
         JUnitTools.assertMatrixEquals(x, ejmlUndampedResult, epsilon);
      }

      System.out.println("Native damped took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (nativeDampedTime / iterations)), 3) + " ms on average");
      System.out.println("Native undamped took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (nativeUndampedTime / iterations)), 3) + " ms on average");
      System.out.println("EJML damped took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (ejmlDampedTime / iterations)), 3) + " ms on average");
      System.out.println("EJML undamped took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (ejmlUndampedTime / iterations)), 3) + " ms on average");
      System.out.println("Average matrix size was " + Precision.round(matrixSizes / iterations, 1));
      System.out.println("Native damped takes " + Precision.round((100.0 * nativeDampedTime / ejmlDampedTime), 0) + "% of EJML time.");
      System.out.println("Native undamped takes " + Precision.round((100.0 * nativeUndampedTime / ejmlUndampedTime), 0) + "% of EJML time.\n");
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
         DenseMatrix64F A = RandomMatrices.createRandom(maxSize, maxSize, random);
         DenseMatrix64F B = RandomMatrices.createRandom(maxSize, maxSize, random);
         DenseMatrix64F C = new DenseMatrix64F(maxSize, maxSize);
         calculator.projectOntoNullspace(A, B, C);
         NativeCommonOps.projectOnNullspace(A, B, C, alpha);
      }

      for (int i = 0; i < iterations; i++)
      {
         int aRows = random.nextInt(maxSize) + 1;
         int bRows = random.nextInt(maxSize) + 1;
         int bCols = random.nextInt(maxSize) + 1;
         matrixSizes += (aRows + bRows + bCols) / 3.0;

         DenseMatrix64F A = RandomMatrices.createRandom(aRows, bCols, random);
         DenseMatrix64F B = RandomMatrices.createRandom(bRows, bCols, random);
         DenseMatrix64F actual = new DenseMatrix64F(aRows, bCols);
         DenseMatrix64F expected = new DenseMatrix64F(aRows, bCols);

         nativeTime -= System.nanoTime();
         NativeCommonOps.projectOnNullspace(A, B, actual, alpha);
         nativeTime += System.nanoTime();

         ejmlTime -= System.nanoTime();
         calculator.projectOntoNullspace(A, B, expected);
         ejmlTime += System.nanoTime();

         JUnitTools.assertMatrixEquals(expected, actual, epsilon);
      }

      System.out.println("Native took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (nativeTime / iterations)), 3) + " ms on average");
      System.out.println("EJML took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (ejmlTime / iterations)), 3) + " ms on average");
      System.out.println("Average matrix size was " + Precision.round(matrixSizes / iterations, 1));
      System.out.println("Native takes " + Precision.round((100.0 * nativeTime / ejmlTime), 0) + "% of EJML time.\n");
   }

   public static void main(String[] args)
   {
      int size = 500;
      Random random = new Random(40L);
      DenseMatrix64F A = RandomMatrices.createRandom(size, size, random);
      DenseMatrix64F B = RandomMatrices.createRandom(size, size, random);
      DenseMatrix64F AtBA = new DenseMatrix64F(size, size);

      System.out.println("Running...");

      while (true)
      {
         NativeCommonOps.multQuad(A, B, AtBA);
      }
   }
}
