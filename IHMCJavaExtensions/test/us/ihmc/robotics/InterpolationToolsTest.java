package us.ihmc.robotics;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.Random;

public class InterpolationToolsTest
{
   private static final double epsilon = 1e-7;
   private static final int iters = 1000;
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
   public void testPiecewiseInterpolation()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 0.2);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 0.8);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 0.5);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 1.1);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 30.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, -0.1);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, -30.0);
         assertEquals(value, boundA, epsilon);

         double alpha = random.nextDouble();
         if (alpha < 0.5)
         {
            value = InterpolationTools.piecewiseInterpolate(boundA, boundB, alpha);
            assertEquals(value, boundA, epsilon);
         }
         else
         {
            value = InterpolationTools.piecewiseInterpolate(boundA, boundB, alpha);
            assertEquals(value, boundB, epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPiecewiseInterpolationRoundDown()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 0.2);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 0.8);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 0.5);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 1.1);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 30.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, -0.1);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, -30.0);
         assertEquals(value, boundA, epsilon);

         double alpha = random.nextDouble();
         if (alpha > 0.5)
         {
            value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, alpha);
            assertEquals(value, boundB, epsilon);
         }
         else
         {
            value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, alpha);
            assertEquals(value, boundA, epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearInterpolateBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.linearInterpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.linearInterpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHermite01InterpolateBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.hermite01Interpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.hermite01Interpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHermiteInterpolateBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();
         double boundATangent = 100.0 * random.nextDouble();
         double boundBTangent = 100.0 * random.nextDouble();

         double value = InterpolationTools.hermiteInterpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.hermiteInterpolate(boundA, boundATangent, boundB, boundBTangent, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.hermiteInterpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.hermiteInterpolate(boundA, boundATangent, boundB, boundBTangent, 1.0);
         assertEquals(value, boundB, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLogisticInterpolateBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.logisticInterpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, 1e-1);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, 1e-1);
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLogisticInterpolateSlopedBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.logisticInterpolate(boundA, boundB, 0.0, 10.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, 1.0, 10.0);
         assertEquals(value, boundB, epsilon);
      }

   }
}
