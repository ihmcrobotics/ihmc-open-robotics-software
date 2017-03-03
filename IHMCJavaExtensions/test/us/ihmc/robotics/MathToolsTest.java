package us.ihmc.robotics;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.Assertions;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class MathToolsTest
{
   private Random random;

   @Before
   public void setUp() throws Exception
   {
      random = new Random(100L);
   }

   @After
   public void tearDown() throws Exception
   {
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor()
           throws NoSuchMethodException, SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      // Screw you clover, I can test the private constructor(!)
      assertEquals(1, MathTools.class.getDeclaredConstructors().length);
      Constructor<MathTools> constructor = MathTools.class.getDeclaredConstructor();
      assertTrue(Modifier.isPrivate(constructor.getModifiers()));
      constructor.setAccessible(true);
      constructor.newInstance();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClampWrongBounds()
   {
      Assertions.assertExceptionThrown(RuntimeException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            double min = 1.0;
            double max = 0.9;
            MathTools.clamp(5.0, min, max);
         }
      });
      Assertions.assertExceptionThrown(RuntimeException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            float min = 1.0f;
            float max = 0.9f;
            MathTools.clamp(5.0, min, max);
         }
      });
      Assertions.assertExceptionThrown(RuntimeException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            int min = 1;
            int max = -1;
            MathTools.clamp(5.0, min, max);
         }
      });
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClamp2()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double max = rand.nextDouble() * 1000.0;
         double clippedVal = MathTools.clamp(max * 2.0, max);
         assertEquals(clippedVal, max, 1e-7);

         max = rand.nextDouble() * 1000.0;
         clippedVal = MathTools.clamp(max * -2.0, max);
         assertEquals(clippedVal, -max, 1e-7);

         max = rand.nextDouble() * 1000.0;
         clippedVal = MathTools.clamp((float) (max * 2.0), (float) max);
         assertEquals(clippedVal, max, 1e-4);

         max = rand.nextDouble() * 1000.0;
         clippedVal = MathTools.clamp((float) (max * -2.0), (float) max);
         assertEquals(clippedVal, -max, 1e-4);
         
         int maxInt = rand.nextInt(1000);
         int clippedInt = MathTools.clamp(maxInt * -2, maxInt);
         assertEquals(clippedInt, -maxInt, 1e-4);
         
         maxInt = rand.nextInt(1000);
         clippedInt = MathTools.clamp(maxInt * 2, maxInt);
         assertEquals(clippedInt, maxInt, 1e-4);
         
         maxInt = rand.nextInt(1000);
         clippedInt = MathTools.clamp(maxInt, maxInt);
         assertEquals(clippedInt, maxInt, 1e-4);
         
         maxInt = rand.nextInt(1000);
         clippedInt = MathTools.clamp(-maxInt, maxInt);
         assertEquals(clippedInt, -maxInt, 1e-4);
         
         maxInt = rand.nextInt(1000);
         clippedInt = MathTools.clamp(-maxInt / 2, maxInt);
         assertEquals(clippedInt, -maxInt / 2, 1e-4);
         
         maxInt = rand.nextInt(1000);
         clippedInt = MathTools.clamp(maxInt / 2, maxInt);
         assertEquals(clippedInt, maxInt / 2, 1e-4);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClampNaN()
   {
      assertTrue(Double.isNaN(MathTools.clamp(Double.NaN, 0.0, 1.0)));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCheckIfInRange()
   {
      assertTrue(MathTools.intervalContains(5, 0, 6));
      assertTrue(MathTools.intervalContains(6, 0, 6));
      assertTrue(MathTools.intervalContains(0, 0, 6));
      assertFalse(MathTools.intervalContains(7, 0, 6));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testCheckIfInRange_2()
   {
      MathTools.checkIntervalContains(-5, -1, 1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClamp()
   {
      for (int i = 0; i < 10; i++)
      {
         double min = random.nextDouble();
         double max = min + random.nextDouble();
         double val = 3.0 * (random.nextDouble() - 0.25);
         double result = MathTools.clamp(val, min, max);

         boolean tooSmall = result < min;
         boolean tooBig = result > max;
         if (tooSmall || tooBig)
         {
            fail();
         }

         boolean withinBounds = (val > min) && (val < max);
         if (withinBounds)
         {
            assertEquals(val, result, 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void squareTest()
   {
      double[] randomValues = RandomNumbers.nextDoubleArray(random, 25, 10.0);
      for (double randomValue : randomValues)
      {
         assertEquals(MathTools.square(randomValue), Math.pow(randomValue, 2), 1e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void cubeTest()
   {
      double[] randomValues = RandomNumbers.nextDoubleArray(random, 25, 10.0);
      for (double randomValue : randomValues)
      {
         assertEquals(MathTools.cube(randomValue), Math.pow(randomValue, 3), 1e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void powWithIntegerTest()
   {
      int numberOfTrials = 10000;
      double[] randomValues = RandomNumbers.nextDoubleArray(random, numberOfTrials, 0.1, 10.0);
      int[] randomExponents = RandomNumbers.nextIntArray(random, numberOfTrials, 10);
      for (int i = 0; i < numberOfTrials; i++)
      {
         double x = randomValues[i];
         int exp = randomExponents[i];
         double xPowedToTest = MathTools.powWithInteger(x, exp);
         double xPowedExpected = Math.pow(x, (double) exp);
         double errorRatio = (xPowedToTest - xPowedExpected) / xPowedExpected;
         boolean isRelativeErrorLowEnough = MathTools.epsilonEquals(errorRatio, 0.0, 1.0e-15);
         assertTrue(isRelativeErrorLowEnough);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void signTest()
   {
      double[] randomValues = RandomNumbers.nextDoubleArray(random, 25, 10.0);
      for (double randomValue : randomValues)
      {
         if (randomValue == 0.0)
            continue;
         assertEquals(Math.signum(randomValue), MathTools.sign(randomValue), 1e-12);
      }

      assertEquals(1.0, MathTools.sign(0.0), 1e-12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void sumDoublesTest()
   {
      double[] posVals = new double[25];
      double[] negVals = new double[25];
      ArrayList<Double> posValsList = new ArrayList<Double>();
      ArrayList<Double> negValsList = new ArrayList<Double>();
      for (int i = 0; i < 25; i++)
      {
         posVals[i] = 1.5;
         negVals[i] = -1.5;
         posValsList.add(1.5);
         negValsList.add(-1.5);
      }

      assertEquals(MathTools.sum(posVals), 37.5, 1e-12);
      assertEquals(MathTools.sum(negVals), -37.5, 1e-12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void sumIntegerTest()
   {
      int[] posVals = new int[25];
      int[] negVals = new int[25];
      ArrayList<Integer> posValsList = new ArrayList<Integer>();
      ArrayList<Integer> negValsList = new ArrayList<Integer>();
      for (int i = 0; i < 25; i++)
      {
         posVals[i] = 1;
         negVals[i] = -1;
         posValsList.add(1);
         negValsList.add(-1);
      }

      assertEquals(MathTools.sum(posVals), 25);
      assertEquals(MathTools.sum(negVals), -25);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void dotPlusTest()
   {
      double[] randomValues = RandomNumbers.nextDoubleArray(random, 25, 10.0);
      int[] randomInts = RandomNumbers.nextIntArray(random, 25, 10);
      double sumOfRandomValues = MathTools.sum(randomValues);
      long sumOfInts = MathTools.sum(randomInts);

      randomValues = MathTools.dotPlus(randomValues, 7.3);
      assertEquals(sumOfRandomValues + 25 * 7.3, MathTools.sum(randomValues), 1e-12);

      randomInts = MathTools.dotPlus(randomInts, 7);
      assertEquals(sumOfInts + 25 * 7, MathTools.sum(randomInts));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsInsideBounds()
   {
      double[] randomValues = RandomNumbers.nextDoubleArray(random, 25, 12.5);
      for (double randomValue : randomValues)
      {
         assertTrue(MathTools.intervalContains(randomValue, -12.5, 12.5, false, false));

         if (randomValue < 0)
            randomValue -= 12.6;
         else
            randomValue += 12.6;

         assertFalse(MathTools.intervalContains(randomValue, -12.5, 12.5, false, false));

      }

      assertFalse(MathTools.intervalContains(Double.NaN, -10.0, 10.0, false, false));

      assertFalse(MathTools.intervalContains(10.0, -10.0, 10.0, false, false));
      assertFalse(MathTools.intervalContains(-10.0, -10.0, 10.0, false, false));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testIsInsideBoundsWrongBounds()
   {
      double min = 1.0;
      double max = 0.9;
      MathTools.intervalContains(5.0, min, max, false, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsInsideBoundsInclusive()
   {
      double[] randomValues = RandomNumbers.nextDoubleArray(random, 25, 12.5);
      for (double randomValue : randomValues)
      {
         assertTrue(MathTools.intervalContains(randomValue, -12.5, 12.5));

         if (randomValue < 0)
            randomValue -= 12.6;
         else
            randomValue += 12.6;

         assertFalse(MathTools.intervalContains(randomValue, -12.5, 12.5));

      }

      assertFalse(MathTools.intervalContains(Double.NaN, -10.0, 10.0));

      assertTrue(MathTools.intervalContains(10.0, -10.0, 10.0));
      assertTrue(MathTools.intervalContains(-10.0, -10.0, 10.0));


      assertTrue(MathTools.intervalContains(5.0, 5.0, 5.0));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testIsInsideBoundsWrongBoundsInclusive()
   {
      double min = 1.0;
      double max = 0.9;
      MathTools.intervalContains(5.0, min, max);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testIsBoundedByMethods()
   {
      Assertions.assertExceptionThrown(RuntimeException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            MathTools.intervalContains(0.0, 1.0, -1.0, false, false);
         }
      });
      assertTrue(MathTools.intervalContains(0.0, -1.0, 1.0, false, false));
      assertFalse(MathTools.intervalContains(-1.0, -1.0, 1.0, false, false));
      assertFalse(MathTools.intervalContains(1.0, -1.0, 1.0, false, false));
      assertTrue(MathTools.intervalContains(-1.0, -1.0, 1.0));
      assertTrue(MathTools.intervalContains(1.0, -1.0, 1.0));
      assertTrue(MathTools.intervalContains(1.0, -1.0, 1.0, 1e-12));
      assertFalse(MathTools.intervalContains(1.0, -1.0, 1.0, 1e-12, false, false));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMin()
   {
      double[] numbers =
      {
         -1.0, -4.0, 4.0, 3.0, 0.0, 1.0, -2.0, -5.0, -3.0, 2.0, 2.0, 3.0, 5.0, 5.0
      };

      assertEquals(MathTools.min(numbers), -5.0, 1e-34);

      numbers[4] = Double.POSITIVE_INFINITY;
      assertFalse(Double.isInfinite(MathTools.min(numbers)));

      numbers[4] = Double.NEGATIVE_INFINITY;
      assertTrue(Double.isInfinite(MathTools.min(numbers)));

      numbers[4] = Double.NaN;
      assertTrue(Double.isNaN(MathTools.min(numbers)));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMax()
   {
      double[] numbers =
      {
         -1.0, -4.0, 4.0, 3.0, 0.0, 1.0, -2.0, -5.0, -3.0, 2.0, 2.0, 3.0, 5.0, 5.0
      };

      assertEquals(MathTools.max(numbers), 5.0, 1e-34);

      numbers[4] = Double.POSITIVE_INFINITY;
      assertTrue(Double.isInfinite(MathTools.max(numbers)));

      numbers[4] = Double.NEGATIVE_INFINITY;
      assertFalse(Double.isInfinite(MathTools.max(numbers)));

      numbers[4] = Double.NaN;
      assertTrue(Double.isNaN(MathTools.max(numbers)));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMeanArray()
   {
      double numbers[] =
      {
         -1.0, -4.0, 4.0, 3.0, 0.0, 1.0, -2.0, -5.0, -3.0, 2.0, 2.0, 3.0, 5.0, 5.0
      };
      assertEquals(0.7143, MathTools.average(numbers), 1e-4);

      assertEquals(5.0, MathTools.average(new double[] {5.0}), 1e-34);

      numbers[4] = Double.POSITIVE_INFINITY;
      assertTrue(Double.isInfinite(MathTools.average(numbers)));

      numbers[4] = Double.NEGATIVE_INFINITY;
      assertTrue(Double.isInfinite(MathTools.average(numbers)));

      numbers[4] = Double.NaN;
      assertTrue(Double.isNaN(MathTools.average(numbers)));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMeanArrayList()
   {
      Double numbersArray[] =
      {
         -1.0, -4.0, 4.0, 3.0, 0.0, 1.0, -2.0, -5.0, -3.0, 2.0, 2.0, 3.0, 5.0, 5.0
      };
      ArrayList<Double> numbers = new ArrayList<Double>(Arrays.asList(numbersArray));
      assertEquals(0.7143, MathTools.average(numbers), 1e-4);

      assertEquals(5.0, MathTools.average(new double[] {5.0}), 1e-34);

      numbers.set(4, Double.POSITIVE_INFINITY);
      assertTrue(Double.isInfinite(MathTools.average(numbers)));

      numbers.set(4, Double.NEGATIVE_INFINITY);
      assertTrue(Double.isInfinite(MathTools.average(numbers)));

      numbers.set(4, Double.NaN);
      assertTrue(Double.isNaN(MathTools.average(numbers)));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testCheckIfInRangeFalse()
   {
      MathTools.checkIntervalContains(5.0, -3.0, 2.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCheckIfInRangeTrue()
   {
      MathTools.checkIntervalContains(1.0, -3.0, 2.0);
      MathTools.checkIntervalContains(5.0, 5.0, 5.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDiff()
   {
      double[] array = {45, -11, 7};
      double[] expectedReturn = {-56, 18};
      double[] actualReturn = MathTools.diff(array);
      assertEquals(expectedReturn[0], actualReturn[0], 1e-12);
      assertEquals(expectedReturn[1], actualReturn[1], 1e-12);

      double[] array2 = {-20, 1, -2.9};
      double[] expectedReturn2 = {21, -3.9};
      double[] actualReturn2 = MathTools.diff(array2);

      assertEquals(expectedReturn2[0], actualReturn2[0], 1e-12);
      assertEquals(expectedReturn2[1], actualReturn2[1], 1e-12);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEpsilonEquals()
   {
      double v1 = 2.0;
      double v2 = 1.0;
      double epsilon = 3.0;
      boolean expectedReturn = true;
      boolean actualReturn = MathTools.epsilonEquals(v1, v2, epsilon);
      assertEquals(expectedReturn, actualReturn);

      v1 = Double.NaN;
      v2 = Double.NaN;
      epsilon = 3.0;
      expectedReturn = true;
      actualReturn = MathTools.epsilonEquals(v1, v2, epsilon);
      assertTrue(actualReturn);

      /** @todo fill in the test code */

      double v3 = 1.0;
      double v4 = 0.0;
      double epsi = 0.0;
      boolean expectedReturn2 = false;
      boolean actualReturn2 = MathTools.epsilonEquals(v3, v4, epsi);
      assertEquals(expectedReturn2, actualReturn2);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPercentEquals()
   {
      double v1 = 1.0;
      double v2 = 1.099;
      double percent = 0.1;
      boolean expectedReturn = true;
      boolean actualReturn = MathTools.withinPercentEquals(v1, v2, percent);
      assertEquals(expectedReturn, actualReturn);

      v1 = 1.0;
      v2 = -1.0;
      percent = 0.01;
      expectedReturn = false;
      actualReturn = MathTools.withinPercentEquals(v1, v2, percent);
      assertEquals(expectedReturn, actualReturn);

      v1 = 1.0;
      v2 = 1.009999;
      percent = 0.01;
      expectedReturn = true;
      actualReturn = MathTools.withinPercentEquals(v1, v2, percent);
      assertEquals(expectedReturn, actualReturn);

      v1 = 1.0;
      v2 = 1.099;
      percent = 0.01;
      expectedReturn = false;
      actualReturn = MathTools.withinPercentEquals(v1, v2, percent);
      assertEquals(expectedReturn, actualReturn);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCheckIsEqual()
   {
      MathTools.checkIfEqual(1, 1);
      MathTools.checkIfEqual(-2, -2);

      MathTools.checkIfEqual(2.0, 2.001, 0.1);
      MathTools.checkIfEqual(-2.0, -2.001, 0.1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testCheckIsEqualNaN()
   {
      MathTools.checkIfEqual(Double.NaN, Double.NaN, 1e-12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testCheckIsEqualInt()
   {
      MathTools.checkIfEqual(2, 4);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsGreaterThan()
   {
      assertTrue(MathTools.isSignificantlyGreaterThan(2.00011000, 2.00010000, 8));
      assertFalse(MathTools.isSignificantlyGreaterThan(2.00011000, 2.00010000, 4));
      
      assertTrue(MathTools.isPreciselyGreaterThan(2.00011000, 2.00010000, 1e-8));
      assertFalse(MathTools.isPreciselyGreaterThan(2.00011000, 2.00010000, 1e-3));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsGreaterThanOrEqualTo()
   {
      assertTrue(MathTools.isSignificantlyGreaterThanOrEqualTo(2.00011000, 2.00010000, 8));
      assertTrue(MathTools.isSignificantlyGreaterThanOrEqualTo(2.00011000, 2.00010000, 4));
      assertTrue(MathTools.isSignificantlyGreaterThanOrEqualTo(2.00019000, 2.00020000, 4));
      assertTrue(MathTools.isSignificantlyGreaterThanOrEqualTo(2.00019000, 2.00020000, 5));
      
      assertTrue(MathTools.isPreciselyGreaterThanOrEqualTo(2.00011000, 2.00010000, 1e-8));
      assertTrue(MathTools.isPreciselyGreaterThanOrEqualTo(2.00011000, 2.00010000, 1e-3));
      assertTrue(MathTools.isPreciselyGreaterThanOrEqualTo(2.00019000, 2.00020000, 1e-3));
      assertTrue(MathTools.isPreciselyGreaterThanOrEqualTo(2.00019000, 2.00020000, 1e-4));
      assertTrue(MathTools.isPreciselyGreaterThanOrEqualTo(2.00020000, 2.00019000, 1e-5));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLessThan()
   {
      assertFalse(MathTools.isPreciselyLessThan(2.00011000, 2.00010000, 1e-8));
      assertFalse(MathTools.isPreciselyLessThan(2.00011000, 2.00010000, 1e-4));
      assertFalse(MathTools.isPreciselyLessThan(2.00019000, 2.00020000, 1e-4));
      assertTrue(MathTools.isPreciselyLessThan(2.00019000, 2.00020000, 1e-5));
      
      assertFalse(MathTools.isSignificantlyLessThan(2.00011000, 2.00010000, 1));
      assertFalse(MathTools.isSignificantlyLessThan(2.00011000, 2.00010000, 5));
      assertFalse(MathTools.isSignificantlyLessThan(2.00019000, 2.00020000, 5));
      assertTrue(MathTools.isSignificantlyLessThan(2.00019000, 2.00020000, 6));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testRoundToPrecision()
   {
      assertEquals("not equal", 100.0, MathTools.roundToPrecision(123.45, 100.0), 0.0);
      assertEquals("not equal", 120.0, MathTools.roundToPrecision(123.45, 10.0), 0.0);
      assertEquals("not equal", 123.0, MathTools.roundToPrecision(123.45, 1.0), 0.0);
      assertEquals("not equal", 123.5, MathTools.roundToPrecision(123.45, 0.1), 0.0);
      assertEquals("not equal", 123.45, MathTools.roundToPrecision(123.45, 0.01), 0.0);
      assertEquals("not equal", 123.45, MathTools.roundToPrecision(123.46, 0.05), 0.0);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testFloorToPrecision()
   {
      assertEquals("not equal", 100.0, MathTools.floorToPrecision(123.45, 100.0), 0.0);
      assertEquals("not equal", 100.0, MathTools.floorToPrecision(193.45, 100.0), 0.0);
      assertEquals("not equal", 120.0, MathTools.floorToPrecision(123.45, 10.0), 0.0);
      assertEquals("not equal", 120.0, MathTools.floorToPrecision(129.45, 10.0), 0.0);
      assertEquals("not equal", 123.0, MathTools.floorToPrecision(123.45, 1.0), 0.0);
      assertEquals("not equal", 123.0, MathTools.floorToPrecision(123.95, 1.0), 0.0);
      assertEquals("not equal", 123.4, MathTools.floorToPrecision(123.45, 0.1), 0.0);
      assertEquals("not equal", 123.45, MathTools.floorToPrecision(123.45, 0.01), 0.0);
      assertEquals("not equal", 123.45, MathTools.floorToPrecision(123.45, 0.05), 0.0);
      assertEquals("not equal", 123.45, MathTools.floorToPrecision(123.49, 0.05), 0.0);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCeilToPrecision()
   {
      assertEquals("not equal", 200.0, MathTools.ceilToPrecision(123.45, 100.0), 0.0);
      assertEquals("not equal", 200.0, MathTools.ceilToPrecision(193.45, 100.0), 0.0);
      assertEquals("not equal", 130.0, MathTools.ceilToPrecision(123.45, 10.0), 0.0);
      assertEquals("not equal", 124.0, MathTools.ceilToPrecision(123.45, 1.0), 0.0);
      assertEquals("not equal", 123.5, MathTools.ceilToPrecision(123.45, 0.1), 0.0);
      assertEquals("not equal", 123.45, MathTools.ceilToPrecision(123.45, 0.01), 0.0);
      assertEquals("not equal", 123.50, MathTools.ceilToPrecision(123.46, 0.05), 0.0);
      assertEquals("not equal", 8e20, MathTools.ceilToPrecision(7.12413651e20, 1e20), 0.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRoundToSignificantFigures()
   {
      assertEquals("not equal", 100.0, MathTools.roundToSignificantFigures(123.45, 1), 0.0);
      assertEquals("not equal", 120.0, MathTools.roundToSignificantFigures(123.45, 2), 0.0);
      assertEquals("not equal", 123.0, MathTools.roundToSignificantFigures(123.45, 3), 0.0);
      assertEquals("not equal", 123.5, MathTools.roundToSignificantFigures(123.45, 4), 0.0);
      assertEquals("not equal", 123.45, MathTools.roundToSignificantFigures(123.45, 5), 0.0);
      assertEquals("not equal", 123.45, MathTools.roundToSignificantFigures(123.45, 6), 0.0);
      
      assertEquals("not equal", 0.0001, MathTools.roundToSignificantFigures(0.00011, 1), 0.0);
      
      System.out.println("Double.MIN_VALUE: " + Double.MIN_VALUE);
      System.out.println("Double.MAX_VALUE: " + Double.MAX_VALUE);
      System.out.println("Integer.MAX_VALUE: " + Integer.MAX_VALUE);
      // test the limits
      assertEquals("not equal", 1.0000000000000001E-307, MathTools.roundToSignificantFigures(1e-307, 0), 0.0);
      assertEquals("not equal", 0.0, MathTools.roundToSignificantFigures(1.79e-308, 0), 0.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLessThanOrEqualTo()
   {
      assertFalse(MathTools.isSignificantlyLessThanOrEqualTo(2.00011000, 2.00010000, 8));
      assertTrue(MathTools.isSignificantlyLessThanOrEqualTo(2.00011000, 2.00010000, 4));
      assertTrue(MathTools.isSignificantlyLessThanOrEqualTo(2.00019000, 2.00020000, 4));
      assertTrue(MathTools.isSignificantlyLessThanOrEqualTo(2.00019000, 2.00020000, 5));
      
      assertFalse(MathTools.isPreciselyLessThanOrEqualTo(2.00011000, 2.00010000, 1e-8));
      assertTrue(MathTools.isPreciselyLessThanOrEqualTo(2.00011000, 2.00010000, 1e-4));
      assertTrue(MathTools.isPreciselyLessThanOrEqualTo(2.00019000, 2.00020000, 1e-4));
      assertTrue(MathTools.isPreciselyLessThanOrEqualTo(2.00019000, 2.00020000, 1e-6));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testCheckIsEqualDouble()
   {
      MathTools.checkIfEqual(2.0, 2.001, 0.0001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGcd()
   {
      Random random = new Random(12890471L);
      for (int i = 0; i < 1000; i++)
      {
         long a = random.nextInt(Integer.MAX_VALUE);
         long b = random.nextInt(Integer.MAX_VALUE);

         long c = MathTools.gcd(a, b);

         assertTrue((a % c == 0) && (b % c == 0));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLcm()
   {
      Random random = new Random(1240898L);
      for (int i = 0; i < 1000; i++)
      {
         long a = random.nextInt(Integer.MAX_VALUE);
         long b = random.nextInt(Integer.MAX_VALUE);

         long c = MathTools.lcm(a, b);

         assertTrue((c % a == 0) && (c % b == 0));
      }

      long c = MathTools.lcm(12, 18, 6, 3, 4);
      assertEquals(36, c);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testLcm_2()
   {
      Random rand = new Random();
      MathTools.lcm(rand.nextLong());
   }

// @DeployableTestMethod(estimatedDuration = 0.1)
// @Test(timeout = 300000)
// public void testDiff1()
// {
//   ArrayList array = null;
//   MathTools.diff(array);
//
//   ArrayList expectedReturn = array;
//   ArrayList actualReturn = mathTools.diff(array);
//   assertTrue("Test Failed", actualReturn[0] == expectedReturn[0]
//              && actualReturn[1] == expectedReturn[1]);
// }

// @DeployableTestMethod(estimatedDuration = 0.1)
// @Test(timeout = 300000)
// public void testDiffWithAlphaFilter()
// {
//    ArrayList array = null;
//    double alpha = 0.0;
//    double dt = 0.0;
//    ArrayList expectedReturn = null;
//    ArrayList actualReturn = mathTools.diffWithAlphaFilter(array, alpha, dt);
//    assertEquals("return value", expectedReturn, actualReturn);
//
// }

// @DeployableTestMethod(estimatedDuration = 0.1)
// @Test(timeout = 300000)
// public void testGetQuaternionFromTransform3D()
// {
//    Transform3D transform3D = null;
//    Quat4d q1 = null;
//    mathTools.getQuaternionFromTransform3D(transform3D, q1);
//
// }

// @DeployableTestMethod(estimatedDuration = 0.1)
// @Test(timeout = 300000)
// public void testLoadTransform() throws IOException
// {
//    BufferedReader bufferedReader = null;
//    Transform3D expectedReturn = null;
//    Transform3D actualReturn = mathTools.loadTransform(bufferedReader);
//    assertEquals("return value", expectedReturn, actualReturn);
//
// }

// @DeployableTestMethod(estimatedDuration = 0.1)
// @Test(timeout = 300000)
// public void testSaveTransform()
// {
//    Transform3D transform3D = null;
//    PrintWriter printWriter = null;
//    mathTools.saveTransform(transform3D, printWriter);
//
// }

// @DeployableTestMethod(estimatedDuration = 0.1)
// @Test(timeout = 300000)
// public void testSplitArrayIntoEqualishParts()
// {
//    ArrayList array = null;
//    int numberOfParts = 0;
//    ArrayList expectedReturn = null;
//    ArrayList actualReturn = mathTools.splitArrayIntoEqualishParts(array, numberOfParts);
//    assertEquals("return value", expectedReturn, actualReturn);
// }

   // Needs to be reimplemented with EJML and without generating garbage. 
//   @DeployableTestMethod(estimatedDuration = 0.0)
//   @Test(timeout = 30000)
//   public void testProjectionOntoPlane()
//   {
//      // test by projecting on plane spanning x,y through z=0.1
//
//      Vector3d p1 = new Vector3d(Math.random(), Math.random(), 0.1);
//      Vector3d p2 = new Vector3d(Math.random(), Math.random(), 0.1);
//      Vector3d p3 = new Vector3d(Math.random(), Math.random(), 0.1);
//
//      Vector3d p = new Vector3d(Math.random(), Math.random(), Math.random());
//
//      Vector3d proj = GeometryTools.getProjectionOntoPlane(p1, p2, p3, p);
//
//      assertEquals(p.getX(), proj.getX(), Double.MIN_VALUE);
//      assertEquals(p.getY(), proj.getY(), Double.MIN_VALUE);
//      assertEquals(0.1, proj.getZ(), 10e-10);
//   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testApplyDeadband()
   {
      assertEquals(MathTools.applyDeadband(1.0, 0.0), 1.0, 1e-12);
      assertEquals(MathTools.applyDeadband(1.0, 5.0), 0.0, 1e-12);
      assertEquals(MathTools.applyDeadband(5.0, 5.0), 0.0, 1e-12);
      assertEquals(MathTools.applyDeadband(-5.0, 5.0), 0.0, 1e-12);
      assertEquals(MathTools.applyDeadband(10.0, 5.0), 5.0, 1e-12);
      assertEquals(MathTools.applyDeadband(-10.0, 5.0), -5.0, 1e-12);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOrderOfMagnitude()
   {
      assertEquals(-2, MathTools.orderOfMagnitude(-0.01));
      assertEquals(-2, MathTools.orderOfMagnitude(0.01));
      assertEquals(-1, MathTools.orderOfMagnitude(0.1));
      assertEquals(-1, MathTools.orderOfMagnitude(0.9));
      assertEquals(0, MathTools.orderOfMagnitude(1.0));
      assertEquals(1, MathTools.orderOfMagnitude(10.0));
      assertEquals(2, MathTools.orderOfMagnitude(100.0));
      assertEquals(3, MathTools.orderOfMagnitude(1000.01));
      assertEquals(3, MathTools.orderOfMagnitude(1000.0));
      assertEquals(4, MathTools.orderOfMagnitude(10000.0));
   }
   
   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(MathTools.class, MathToolsTest.class);
   }
}
