package us.ihmc.robotics.numericalMethods;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class DifferentiatorTest
{
   private Differentiator differentiator;
   private Random random;
   private double dt;

   @Before
   public void setUp() throws Exception
   {
      dt = 0.001;
      differentiator = new Differentiator(dt);
      random = new Random(21L);
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSinusDifferentiatedIsCloseToCosinus()
   {
      differentiator = new Differentiator(dt);

      double randomAmplitude = random.nextDouble();
      double randomFrequency = random.nextDouble();
      double delta = 1e-4;

      for (double t = 0.0; t <= 1.0; t = t + dt)
      {
         double functionValue = randomAmplitude * Math.sin(randomFrequency * t);
         double expectedDerivationValue = randomAmplitude * randomFrequency * Math.cos(randomFrequency * t);

         differentiator.update(functionValue);

         if (isAtLeastTwoTimesIterated(t))
         {
            assertEquals(expectedDerivationValue, differentiator.val(), delta);
         }

      }

   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNOrderPolynomDifferentiatedIsNMinusOneOrderPolynom()
   {
      int n_max = 3;
      double[] delta = new double[n_max];
      delta[1] = 1e-2;
      delta[2] = 5e-2;

      for (int i = 1; i < n_max; i++)
      {
         int n = i + 1;

         for (double t = 0.0; t <= 10.0; t = t + dt)
         {
            double functionValue = Math.pow(t, n);
            double expectedDerivationValue = n * Math.pow(t, (n - 1));    // MathTools.square(t);

            differentiator.update(functionValue);

            if (isAtLeastTwoTimesIterated(t))
            {
               assertEquals(expectedDerivationValue, differentiator.val(), delta[i]);
            }

         }
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFirstUpdatedToZero()
   {
      double delta = 1e-12;

      double randomDouble = random.nextDouble();
      differentiator.update(randomDouble);
      assertEquals(0.0, differentiator.val(), delta);
   }

// @Test(timeout=300000,expected = RuntimeException.class)
   @Ignore
   public void testTimeIntervalEqualZero()
   {
      dt = 0.0;
      new Differentiator(dt);

      // TODO 20120911 Khai-Long Ho Hoang: Differentiator class must be enhanced by a detection if time interval dt is equal zero.
   }

// @Test(timeout=300000,expected = RuntimeException.class)
   @Ignore
   public void testTimeIntervalLessZero()
   {
      dt = -random.nextDouble();

      new Differentiator(dt);

      // TODO 20120911 Khai-Long Ho Hoang: Differentiator class must be enhanced by a detection if time interval dt is less than zero.
   }

   private boolean isAtLeastTwoTimesIterated(double t)
   {
      return t >= 2.0 * dt;
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testResetReturnsZero()
   {
      double delta = 1e-12;

      double randomInput = random.nextDouble();
      differentiator.reset(randomInput);
      assertEquals(0.0, differentiator.val(), delta);
   }

}
