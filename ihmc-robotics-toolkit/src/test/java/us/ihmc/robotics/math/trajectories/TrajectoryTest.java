package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.commons.MathTools;

public class TrajectoryTest
{
   private static double EPSILON = 1e-6;

   String namePrefix = "TrajectoryTest";
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearSet()
   {
      Trajectory traj = new Trajectory(2);
      assertEquals(0, traj.getNumberOfCoefficients());
      traj.setLinear(1, 2, 3, 5);
      assertEquals(1, traj.getInitialTime(), Epsilons.ONE_BILLIONTH);
      assertEquals(2, traj.getFinalTime(), Epsilons.ONE_BILLIONTH);
      traj.compute(traj.getInitialTime());
      assertEquals(3.0, traj.getPosition(), Epsilons.ONE_BILLIONTH);
      traj.compute(traj.getFinalTime());
      assertEquals(5.0, traj.getPosition(), Epsilons.ONE_BILLIONTH);
      assertEquals(2, traj.getCoefficient(1), Epsilons.ONE_BILLIONTH);
      assertEquals(1, traj.getCoefficient(0), Epsilons.ONE_BILLIONTH);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearDerivativePointManual()
   {
      //linear polynomial: y(x) = a0 + a1*x
      int numberOfCoefficients = 2;
      Trajectory linear = new Trajectory(numberOfCoefficients);

      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;

      linear.setLinear(x0, xf, y0, yf);

      double x = 2.0 / 3.0 * (xf - x0);
      double a0 = linear.getCoefficient(0);
      double a1 = linear.getCoefficient(1);

      double yLinear = linear.getDerivative(0, x);
      double yManual = a0 + a1 * x;
      assertEquals(yLinear, yManual, EPSILON);

      double dyLinear = linear.getDerivative(1, x);
      double dyManual = a1;
      assertEquals(dyLinear, dyManual, EPSILON);

      double ddyLinear = linear.getDerivative(2, x);
      double ddyManual = 0.0;
      assertEquals(ddyLinear, ddyManual, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearDerivativePointAutomated()
   {
      //linear polynomial: y(x) = a0 + a1*x
      int numberOfCoefficients = 2;
      Trajectory linear = new Trajectory(numberOfCoefficients);

      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;

      linear.setLinear(x0, xf, y0, yf);

      double x = 2.0 / 3.0 * (xf - x0);

      compareDerivativesPoint(linear, x);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCubicDerivativePointAutomated()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      int numberOfCoefficients = 4;
      Trajectory cubic = new Trajectory(numberOfCoefficients);

      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;
      double dy0 = -0.5, dyf = 2.0;

      cubic.setCubic(x0, xf, y0, dy0, yf, dyf);

      double x = 2.0 / 3.0 * (xf - x0);

      compareDerivativesPoint(cubic, x);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testXPowersDerivativeVectorCubic()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      int numberOfCoefficients = 4;
      Trajectory cubic = new Trajectory(numberOfCoefficients);

      int numTrials = 9;
      for (int i = 0; i < numTrials; i++)
      {
         double scaleX0 = 1.0 / Math.random(), scaleXf = 1.0 / Math.random();
         double scaleY0 = 1.0 / Math.random(), scaleYf = 1.0 / Math.random();
         double scaleDY0 = 1.0 / Math.random(), scaleDYf = 1.0 / Math.random();

         double x0 = Math.signum(Math.random()) * Math.random() * scaleX0, xf = x0 + Math.random() * scaleXf;
         double y0 = Math.signum(Math.random()) * Math.random() * scaleY0, yf = Math.signum(Math.random()) * Math.random() * scaleYf;
         double dy0 = Math.signum(Math.random()) * Math.random() * scaleDY0, dyf = Math.signum(Math.random()) * Math.random() * scaleDYf;

         cubic.setCubic(x0, xf, y0, dy0, yf, dyf);

         double x = Math.random() * (xf - x0);

         compareXPowersDerivativesVector(cubic, x);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDerivativeCoefficients()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      int numberOfCoefficients = 8;
      Trajectory septic = new Trajectory(numberOfCoefficients);

      double x0 = 1.0, x1 = 1.2, x2 = 1.9, xf = 2.0;
      double y0 = 0.5, y1 = -0.75, y2 = 1.3, yf = 1.5;
      double dy0 = -0.5, dy1 = 0.5, dy2 = 1.0, dyf = 2.0;

      septic.setSeptic(x0, x1, x2, xf, y0, dy0, y1, dy1, dy2, dy2, yf, dyf);

      int order3Exponent1Func = septic.getCoefficientMultiplierForDerivative(3, 1);
      int order3Exponent1Hand = 0;
      assertEquals(order3Exponent1Func, order3Exponent1Hand, EPSILON);

      int order6Exponent7Func = septic.getCoefficientMultiplierForDerivative(6, 7);
      int order6Exponent7Hand = 5040;
      assertEquals(order6Exponent7Func, order6Exponent7Hand, EPSILON);

      int order0Exponent5Func = septic.getCoefficientMultiplierForDerivative(0, 5);
      int order0Exponent5Hand = 1;
      assertEquals(order0Exponent5Func, order0Exponent5Hand, EPSILON);

      int order3Exponent4Func = septic.getCoefficientMultiplierForDerivative(3, 4);
      int order3Exponent4Hand = 24;
      assertEquals(order3Exponent4Func, order3Exponent4Hand, EPSILON);

      int order5Exponent2Func = septic.getCoefficientMultiplierForDerivative(5, 2);
      int order5Exponent2Hand = 0;
      assertEquals(order5Exponent2Func, order5Exponent2Hand, EPSILON);

      int order1Exponent5Func = septic.getCoefficientMultiplierForDerivative(1, 5);
      int order1Exponent5Hand = 5;
      assertEquals(order1Exponent5Func, order1Exponent5Hand, EPSILON);

      int order11Exponent1Func = septic.getCoefficientMultiplierForDerivative(11, 1);
      int order11Exponent1Hand = 0;
      assertEquals(order11Exponent1Func, order11Exponent1Hand, EPSILON);

      int order13Exponent8Func = septic.getCoefficientMultiplierForDerivative(13, 8);
      int order13Exponent8Hand = 0;
      assertEquals(order13Exponent8Func, order13Exponent8Hand, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDerivativeVersionsCubic()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      int numberOfCoefficients = 4;
      Trajectory cubic = new Trajectory(numberOfCoefficients);

      int numTrials = 9;
      for (int i = 0; i < numTrials; i++)
      {
         double scaleX0 = 1.0 / Math.random(), scaleXf = 1.0 / Math.random();
         double scaleY0 = 1.0 / Math.random(), scaleYf = 1.0 / Math.random();
         double scaleDY0 = 1.0 / Math.random(), scaleDYf = 1.0 / Math.random();

         double x0 = Math.signum(Math.random()) * Math.random() * scaleX0, xf = x0 + Math.random() * scaleXf;
         double y0 = Math.signum(Math.random()) * Math.random() * scaleY0, yf = Math.signum(Math.random()) * Math.random() * scaleYf;
         double dy0 = Math.signum(Math.random()) * Math.random() * scaleDY0, dyf = Math.signum(Math.random()) * Math.random() * scaleDYf;

         cubic.setCubic(x0, xf, y0, dy0, yf, dyf);

         double x = Math.random() * (xf - x0);

         compareDerivativeVersions(cubic, x);
      }
   }

   public void compareDerivativesPoint(Trajectory polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for (int i = 0; i < coefficients.length + 3; i++)
      {
         double generalizedDYPoly = polynomial.getDerivative(i, x);

         double generalizedDYHand = 0.0;
         if (i < coefficients.length)
         {
            for (int j = i; j < coefficients.length; j++)
            {
               double derivativeCoefficient = polynomial.getCoefficientMultiplierForDerivative(i, j);
               generalizedDYHand += coefficients[j] * derivativeCoefficient * Math.pow(x, j - i);
            }
         }
         else
         {
            generalizedDYHand = 0.0;
         }
         assertEquals(generalizedDYPoly, generalizedDYHand, EPSILON);
      }
   }

   public void compareXPowersDerivativesVector(Trajectory polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for (int i = 0; i < coefficients.length + 3; i++)
      {
         DenseMatrix64F generalizedDYPoly = polynomial.getXPowersDerivativeVector(i, x);
         DenseMatrix64F generalizedDYHand = new DenseMatrix64F(generalizedDYPoly.getNumRows(), generalizedDYPoly.getNumCols());
         if (i < coefficients.length)
         {
            for (int j = i; j < coefficients.length; j++)
            {
               double derivativeCoefficient = polynomial.getCoefficientMultiplierForDerivative(i, j);
               generalizedDYHand.set(j, derivativeCoefficient * Math.pow(x, j - i));
            }
         }
         for (int k = 0; k < coefficients.length; k++)
         {
            assertEquals(generalizedDYPoly.get(k), generalizedDYHand.get(k), EPSILON);
         }
      }
   }

   public void compareDerivativeVersions(Trajectory polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for (int i = 0; i < coefficients.length + 3; i++)
      {
         double generalizedDYPolyScalar = polynomial.getDerivative(i, x);
         double generalizedDYHandScalar = 0.0;

         DenseMatrix64F generalizedDYPolyVector = polynomial.getXPowersDerivativeVector(i, x);
         for (int j = 0; j < generalizedDYPolyVector.numCols; j++)
         {
            generalizedDYHandScalar += generalizedDYPolyVector.get(j) * coefficients[j];
         }
         assertEquals(generalizedDYPolyScalar, generalizedDYHandScalar, EPSILON);
      }
   }

   @Test(timeout = 30000)
   public void testGetDerivative()
   {
      Trajectory traj = new Trajectory(10);
      Trajectory dervTraj = new Trajectory(9);
      traj.setCubic(1, 10, 0, 8);
      traj.getDerivative(dervTraj, 1);

      assert (dervTraj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(1.0 * traj.getCoefficient(1), dervTraj.getCoefficient(0), 1));
      assert (MathTools.epsilonCompare(2.0 * traj.getCoefficient(2), dervTraj.getCoefficient(1), 1));
      assert (MathTools.epsilonCompare(3.0 * traj.getCoefficient(3), dervTraj.getCoefficient(2), 1));
   }
}