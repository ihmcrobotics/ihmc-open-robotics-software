package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import java.util.Random;

public class TrajectoryTest
{
   private static double epsilon = 1e-2;

   String namePrefix = "TrajectoryTest";
   private final Random random = new Random(3294508L);

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
      assertEquals(yLinear, yManual, epsilon);

      double dyLinear = linear.getDerivative(1, x);
      double dyManual = a1;
      assertEquals(dyLinear, dyManual, epsilon);

      double ddyLinear = linear.getDerivative(2, x);
      double ddyManual = 0.0;
      assertEquals(ddyLinear, ddyManual, epsilon);
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
   public void testTimeScaling()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      Random random = new Random(1738L);
      int numberOfCoefficients = 4;
      Trajectory trajectory = new Trajectory(numberOfCoefficients);

      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
      {
         double t0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double tf = RandomNumbers.nextDouble(random, t0, t0 + 10.0);
         double x0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xf = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xd0 = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double xdf = RandomNumbers.nextDouble(random, -100.0, 100.0);

         double[] initialValues = new double[]{x0, xd0};
         double[] finalValues = new double[]{xf, xdf};

         trajectory.setCubic(t0, tf, x0, xd0, xf, xdf);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tf), epsilon);

         double t0New = RandomNumbers.nextDouble(random, tf - 10.0, tf);
         trajectory.setInitialTimeMaintainingBounds(t0New);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0New), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tf), epsilon);

         double tfNew = RandomNumbers.nextDouble(random, t0New, t0New + 10.0);
         trajectory.setFinalTimeMaintainingBounds(tfNew);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0New), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tfNew), epsilon);
      }

      numberOfCoefficients = 5;
      trajectory = new Trajectory(numberOfCoefficients);

      for (int iter = 0; iter < iters; iter++)
      {
         double t0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double tf = RandomNumbers.nextDouble(random, t0, t0 + 10.0);
         double x0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xf = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xd0 = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double xdf = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double xdd0 = RandomNumbers.nextDouble(random, -1000.0, 1000.0);

         double[] initialValues = new double[]{x0, xd0, xdd0};
         double[] finalValues = new double[]{xf, xdf};
         trajectory.setQuartic(t0, tf, x0, xd0, xdd0, xf, xdf);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tf), epsilon);


         double t0New = RandomNumbers.nextDouble(random, tf - 10.0, tf);
         trajectory.setInitialTimeMaintainingBounds(t0New);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0New), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tf), epsilon);

         double tfNew = RandomNumbers.nextDouble(random, t0New, t0New + 10.0);
         trajectory.setFinalTimeMaintainingBounds(tfNew);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0New), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tfNew), epsilon);
      }


      numberOfCoefficients = 6;
      trajectory = new Trajectory(numberOfCoefficients);

      for (int iter = 0; iter < iters; iter++)
      {
         double t0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double tf = RandomNumbers.nextDouble(random, t0, t0 + 10.0);
         double x0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xf = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xd0 = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double xdf = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double xdd0 = RandomNumbers.nextDouble(random, -1000.0, 1000.0);
         double xddf = RandomNumbers.nextDouble(random, -1000.0, 1000.0);

         double[] initialValues = new double[]{x0, xd0, xdd0};
         double[] finalValues = new double[]{xf, xdf, xddf};
         trajectory.setQuintic(t0, tf, x0, xd0, xdd0, xf, xdf, xddf);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals("Order " + i + " of iter " + iter + " is wrong.", initialValues[i], trajectory.getDerivative(i, t0), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals("Order " + i + " of iter " + iter + " is wrong.", finalValues[i], trajectory.getDerivative(i, tf), epsilon);


         double t0New = RandomNumbers.nextDouble(random, tf - 10.0, tf);
         trajectory.setInitialTimeMaintainingBounds(t0New);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0New), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tf), epsilon);

         double tfNew = RandomNumbers.nextDouble(random, t0New, t0New + 10.0);
         trajectory.setFinalTimeMaintainingBounds(tfNew);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0New), epsilon);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tfNew), epsilon);
      }

      numberOfCoefficients = 7;
      trajectory = new Trajectory(numberOfCoefficients);

      for (int iter = 0; iter < iters; iter++)
      {
         double t0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double tf = RandomNumbers.nextDouble(random, t0, t0 + 10.0);
         double x0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xf = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xd0 = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double xdf = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double xdd0 = RandomNumbers.nextDouble(random, -1000.0, 1000.0);
         double xddf = RandomNumbers.nextDouble(random, -1000.0, 1000.0);
         double xm0 = RandomNumbers.nextDouble(random, -10, 10.0);

         double[] initialValues = new double[]{x0, xd0, xdd0};
         double[] finalValues = new double[]{xf, xdf, xddf};
         trajectory.setSexticUsingWaypoint(t0, 0.5 * (tf + t0), tf, x0, xd0, xdd0, xm0, xf, xdf, xddf);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0),  1e-1);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tf), 1e-1);


         double t0New = RandomNumbers.nextDouble(random, tf - 10.0, tf);
         trajectory.setInitialTimeMaintainingBounds(t0New);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0New), 1e-1);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tf), 1e-1);

         double tfNew = RandomNumbers.nextDouble(random, t0New, t0New + 10.0);
         trajectory.setFinalTimeMaintainingBounds(tfNew);

         for (int i = 0; i < initialValues.length; i++)
            assertEquals(initialValues[i], trajectory.getDerivative(i, t0New), 1e-1);
         for (int i = 0; i < finalValues.length; i++)
            assertEquals(finalValues[i], trajectory.getDerivative(i, tfNew), 1e-1);
      }
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
   public void testEvaluateGeometricSequenceDerivativeForRandomInputs()
   {
      int maxNumberOfCoefficients = 8;
      int iterations = 50;
      for (int i = 0; i < iterations; i++)
      {
         int numberOfCoefficients = 1 + random.nextInt(maxNumberOfCoefficients);
         Trajectory trajectory = new Trajectory(numberOfCoefficients);
         trajectory.setDirectly(new double[numberOfCoefficients]);

         int derivativeOrder = 1 + random.nextInt(numberOfCoefficients);
         double x0 = EuclidCoreRandomTools.nextDouble(random, 5.0);

         DenseMatrix64F derivativeElements = trajectory.evaluateGeometricSequenceDerivative(derivativeOrder, x0);
         for (int j = 0; j < derivativeOrder; j++)
         {
            Assert.assertEquals(derivativeElements.get(j), 0.0, epsilon);
         }

         for (int j = numberOfCoefficients - 1; j >= derivativeOrder; j--)
         {
            int exponent = j - derivativeOrder;
            double expectedDerivativeElement = Math.pow(x0, exponent);
            for (int k = 1; k <= derivativeOrder; k++)
            {
               expectedDerivativeElement *= (exponent + k);
            }

            double derivativeElement = derivativeElements.get(j);
            Assert.assertEquals(expectedDerivativeElement, derivativeElement, epsilon);
         }
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

      int order3Exponent1Func = getCoefficientMultiplierForDerivative(3, 1);
      int order3Exponent1Hand = 0;
      assertEquals(order3Exponent1Func, order3Exponent1Hand, epsilon);

      int order6Exponent7Func = getCoefficientMultiplierForDerivative(6, 7);
      int order6Exponent7Hand = 5040;
      assertEquals(order6Exponent7Func, order6Exponent7Hand, epsilon);

      int order0Exponent5Func = getCoefficientMultiplierForDerivative(0, 5);
      int order0Exponent5Hand = 1;
      assertEquals(order0Exponent5Func, order0Exponent5Hand, epsilon);

      int order3Exponent4Func = getCoefficientMultiplierForDerivative(3, 4);
      int order3Exponent4Hand = 24;
      assertEquals(order3Exponent4Func, order3Exponent4Hand, epsilon);

      int order5Exponent2Func = getCoefficientMultiplierForDerivative(5, 2);
      int order5Exponent2Hand = 0;
      assertEquals(order5Exponent2Func, order5Exponent2Hand, epsilon);

      int order1Exponent5Func = getCoefficientMultiplierForDerivative(1, 5);
      int order1Exponent5Hand = 5;
      assertEquals(order1Exponent5Func, order1Exponent5Hand, epsilon);

      int order11Exponent1Func = getCoefficientMultiplierForDerivative(11, 1);
      int order11Exponent1Hand = 0;
      assertEquals(order11Exponent1Func, order11Exponent1Hand, epsilon);

      int order13Exponent8Func = getCoefficientMultiplierForDerivative(13, 8);
      int order13Exponent8Hand = 0;
      assertEquals(order13Exponent8Func, order13Exponent8Hand, epsilon);
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
               double derivativeCoefficient = getCoefficientMultiplierForDerivative(i, j);
               generalizedDYHand += coefficients[j] * derivativeCoefficient * Math.pow(x, j - i);
            }
         }
         else
         {
            generalizedDYHand = 0.0;
         }
         assertEquals(generalizedDYPoly, generalizedDYHand, epsilon);
      }
   }

   public void compareXPowersDerivativesVector(Trajectory polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for (int i = 0; i < coefficients.length + 3; i++)
      {
         DenseMatrix64F generalizedDYPoly = polynomial.evaluateGeometricSequenceDerivative(i, x);
         DenseMatrix64F generalizedDYHand = new DenseMatrix64F(generalizedDYPoly.getNumRows(), generalizedDYPoly.getNumCols());
         if (i < coefficients.length)
         {
            for (int j = i; j < coefficients.length; j++)
            {
               double derivativeCoefficient = getCoefficientMultiplierForDerivative(i, j);
               generalizedDYHand.set(j, derivativeCoefficient * Math.pow(x, j - i));
            }
         }
         for (int k = 0; k < coefficients.length; k++)
         {
            assertEquals(generalizedDYPoly.get(k), generalizedDYHand.get(k), epsilon);
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

         DenseMatrix64F generalizedDYPolyVector = polynomial.evaluateGeometricSequenceDerivative(i, x);
         for (int j = 0; j < generalizedDYPolyVector.numCols; j++)
         {
            generalizedDYHandScalar += generalizedDYPolyVector.get(j) * coefficients[j];
         }
         assertEquals(generalizedDYPolyScalar, generalizedDYHandScalar, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testQuinticTrajectory()
   {
      Trajectory quinticTrajectory = new Trajectory(-10.0, 10.0, new double[] {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

      quinticTrajectory.setQuintic(0.0, 1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

      quinticTrajectory.compute(0.0);
      assertEquals(quinticTrajectory.getPosition(), 1.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), 2.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      assertEquals(quinticTrajectory.getPosition(), 4.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), 5.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);

      quinticTrajectory.setQuintic(-1.0, 1.0, 1.0, -2.0, 3.0, -4.0, -5.0, 6.0);

      quinticTrajectory.compute(-1.0);
      assertEquals(quinticTrajectory.getPosition(), 1.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), -2.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      assertEquals(quinticTrajectory.getPosition(), -4.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), -5.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);
   }

   private static int getCoefficientMultiplierForDerivative(int order, int exponent)
   {
      int coeff = 1;
      for (int i = exponent; i > exponent - order; i--)
      {
         coeff *= i;
      }
      return coeff;
   }
}