package us.ihmc.robotics.linearAlgebra.commonOps;

import java.util.Random;

import org.apache.commons.math3.util.Precision;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.robotics.testing.JUnitTools;

public class NativeCommonOpsTest
{
   @Test
   public void testLoading()
   {
      Assert.assertTrue("Was not able to load native matrix operations.", NativeCommonOps.useNativeOps);
   }

   @Test
   public void testMult()
   {
      Random random = new Random(40L);
      double epsilon = 1.0e-12;
      int iterations = 3000;
      int maxSize = 100;

      System.out.println("Testing matrix multiplications with random matrices up to size " + maxSize + "...");
      if (!NativeCommonOps.useNativeOps)
      {
         System.err.println("Native operations are not loaded.");
      }

      long nativeTime = 0;
      long ejmlTime = 0;

      for (int i = 0; i < iterations; i++)
      {
         int aRows = random.nextInt(maxSize) + 1;
         int aCols = random.nextInt(maxSize) + 1;
         int bCols = random.nextInt(maxSize) + 1;

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
   }
}
