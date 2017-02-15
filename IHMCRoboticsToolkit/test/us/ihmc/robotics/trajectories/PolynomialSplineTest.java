package us.ihmc.robotics.trajectories;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

public class PolynomialSplineTest
{
   private YoVariableRegistry registry;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("test");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetPosition()
   {
      Random random = new Random(165L);
      int order = 5;
      YoPolynomial spline = new YoPolynomial("test", order, registry);
      double[] coefficients = getRandomCoefficients(order, random);
      spline.setDirectly(coefficients);
      double x = random.nextDouble();
      spline.compute(x);
      double y = spline.getPosition();
      double yCheck = coefficients[0] + coefficients[1] * x + coefficients[2] * x * x + coefficients[3] * x * x * x + coefficients[4] * x * x * x * x;
      assertEquals(yCheck, y, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetVelocity()
   {
      Random random = new Random(1675L);
      int order = 5;
      YoPolynomial spline = new YoPolynomial("test", order, registry);
      double[] coefficients = getRandomCoefficients(order, random);
      spline.setDirectly(coefficients);

      double x = random.nextDouble();
      double dx = 1e-9;
      spline.compute(x);
      double yx = spline.getPosition();
      double dydx = spline.getVelocity();
      spline.compute(x + dx);
      double yxPlusdx = spline.getPosition();
      double dydxNumerical = (yxPlusdx - yx) / dx;
      assertEquals(dydxNumerical, dydx, 1e-6);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetAcceleration()
   {
      Random random = new Random(1675L);
      int order = 5;
      YoPolynomial spline = new YoPolynomial("test", order, registry);
      double[] coefficients = getRandomCoefficients(order, random);
      spline.setDirectly(coefficients);

      double x = random.nextDouble();
      double dx = 1e-9;
      spline.compute(x);
      double d2ydx2 = spline.getAcceleration();
      double dydxx = spline.getVelocity();
      spline.compute(x + dx);
      double dydxxPlusdx = spline.getVelocity();
      double d2ydx2Numerical = (dydxxPlusdx - dydxx) / dx;
      assertEquals(d2ydx2Numerical, d2ydx2, 1e-6);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetIntegral()
   {
      int order = 5;
      YoPolynomial spline = new YoPolynomial("test", order, registry);
      double[] coefficients = new double[]{2.0, 6.0, 9.0, 4.0, 10.0};
      spline.setDirectly(coefficients);
      
      double from = 1.0;
      double to = 2.0;
      double expected = 109.0;
      double actual = spline.getIntegral(from, to);
      assertEquals(expected, actual, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetBasedOnMidPoint()
   {
      Random random = new Random(1635L);
      YoPolynomial spline = new YoPolynomial("test", 5, registry);
      double x0 = random.nextDouble();
      double xFinal = x0 + random.nextDouble();
      double xMid = x0 + (xFinal - x0) / 2.0;
      double z0 = random.nextDouble();
      double zd0 = random.nextDouble();
      double zMid = random.nextDouble();
      double zFinal = random.nextDouble();
      double zdFinal = random.nextDouble();
      spline.setQuarticUsingMidPoint(x0, xFinal, z0, zd0, zMid, zFinal, zdFinal);

      double epsilon = 1e-6;
      spline.compute(x0);
      assertEquals(z0, spline.getPosition(), epsilon);
      assertEquals(zd0, spline.getVelocity(), epsilon);

      spline.compute(xMid);
      assertEquals(zMid, spline.getPosition(), epsilon);

      spline.compute(xFinal);
      assertEquals(zFinal, spline.getPosition(), epsilon);
      assertEquals(zdFinal, spline.getVelocity(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetBasedOnFinalAcceleration()
   {
      Random random = new Random(1635L);
      YoPolynomial spline = new YoPolynomial("test", 5, registry);
      double x0 = random.nextDouble();
      double xFinal = x0 + random.nextDouble();
      double z0 = random.nextDouble();
      double zd0 = random.nextDouble();
      double zFinal = random.nextDouble();
      double zdFinal = random.nextDouble();
      double zddFinal = random.nextDouble();
      spline.setQuarticUsingFinalAcceleration(x0, xFinal, z0, zd0, zFinal, zdFinal, zddFinal);

      double epsilon = 1e-6;
      spline.compute(x0);
      assertEquals(z0, spline.getPosition(), epsilon);
      assertEquals(zd0, spline.getVelocity(), epsilon);

      spline.compute(xFinal);
      assertEquals(zFinal, spline.getPosition(), epsilon);
      assertEquals(zdFinal, spline.getVelocity(), epsilon);
      assertEquals(zddFinal, spline.getAcceleration(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstant()
   {
      Random random = new Random(1635L);
      YoPolynomial spline = new YoPolynomial("test", 5, registry);
      double z = random.nextDouble();
      spline.setConstant(z);
      double[] coefficients = spline.getCoefficients();
      double epsilon = 1e-9;
      assertEquals(z, coefficients[0], epsilon);

      for (int i = 1; i < coefficients.length; i++)
      {
         assertEquals(0.0, coefficients[i], epsilon);
      }
   }

   private double[] getRandomCoefficients(int order, Random random)
   {
      double[] coefficients = new double[order];
      for (int i = 0; i < coefficients.length; i++)
      {
         coefficients[i] = random.nextDouble();
      }

      return coefficients;
   }
}
