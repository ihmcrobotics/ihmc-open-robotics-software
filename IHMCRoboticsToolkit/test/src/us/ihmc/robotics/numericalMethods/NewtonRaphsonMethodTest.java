package us.ihmc.robotics.numericalMethods;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class NewtonRaphsonMethodTest
{
   private static final boolean VERBOSE = false;

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testThirdOrderPolynomial()
   {
      int maxIterations = 100;
      double epsilon = 1e-7;

      NewtonRaphsonMethod newt = new NewtonRaphsonMethod(maxIterations, epsilon);

      double x = 0.0;
      while (!newt.stop())
      {
         newt.update(x, thirdOrderPolynomial(x));
         x = newt.nextX();
      }

      if (VERBOSE)
      {
         System.out.println("Number of iterations required for some third order polynomial: " + newt.getIterations());
         System.out.println("Root found: " + x);
      }
      
      assertEpsilonEquals(thirdOrderPolynomial(x), 0.0, epsilon);
   }

//   @Test(timeout=300000)
//   public void testWeirdFunction()
//   {
//      int maxIterations = 100;
//      double epsilon = 1e-7;
//
//      double r1 = 5.0;
//      double r2 = 7.0;
//
//      NewtonRaphsonMethod newt = new NewtonRaphsonMethod(maxIterations, epsilon);
//
//      double lambda2 = 300.0;
//      while (!newt.stop())
//      {
//         newt.update(lambda2, weirdFunction(lambda2, r1, r2));
//         lambda2 = newt.nextX();
//      }
//
//      System.out.println("Number of iterations required for some weird function: " + newt.getIterations());
//      System.out.println("Root found: " + lambda2);
//      assertEpsilonEquals(weirdFunction(lambda2, r1, r2), 0.0, epsilon);
//   }

   private double thirdOrderPolynomial(double x)
   {
      double a0 = 1.0;
      double a1 = 3.0;
      double a2 = -2.0;
      double a3 = 4.0;

      double ret = a0 + a1 * x + a2 * x * x + a3 * x * x * x;

      return ret;
   }

   @SuppressWarnings("unused")
   private double weirdFunction(double lambda2, double r1, double r2)
   {
      double alpha = 1.0;

      double lambda1 = (r1 * lambda2) / (r2 * (1.0 - Math.exp(-lambda2)) - lambda2);
      double A = r1 / lambda1;
      double B = r2 / (lambda2 * Math.exp(lambda2));
      double C = -(A + B);

      double ret = A * Math.exp(lambda1 * alpha) + B * Math.exp(lambda2 * alpha) + C - 1.0;

      return ret;
   }

   private void assertEpsilonEquals(double value, double checkValue, double epsilon)
   {
      assertTrue("Function value is not approximately zero, value = " + value + ", checkValue = " + checkValue, Math.abs(value - checkValue) < epsilon);
   }
}
