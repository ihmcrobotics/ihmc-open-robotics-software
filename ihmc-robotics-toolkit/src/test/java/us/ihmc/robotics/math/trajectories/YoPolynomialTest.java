package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.fail;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Assertions;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

public class YoPolynomialTest
{
   private static final int ITERATIONS = 1000;
   private static double EPSILON = 1e-6;
   
   String namePrefix = "YoPolynomialTest";

   @Test
   public void testLinearDerivativePointManual()
   {
      //linear polynomial: y(x) = a0 + a1*x
      YoRegistry registry = new YoRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoPolynomial linear = new YoPolynomial(namePrefix + "Linear", numberOfCoefficients, registry);
      
      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;
      
      linear.setLinear(x0, xf, y0, yf);
      
      double x = 2.0/3.0 * (xf - x0);
      double a0 = linear.getCoefficient(0);
      double a1 = linear.getCoefficient(1);
      
      double yLinear = linear.getDerivative(0, x);
      double yManual = a0 + a1*x;
      assertEquals(yLinear, yManual, EPSILON);
            
      double dyLinear = linear.getDerivative(1, x);
      double dyManual = a1;
      assertEquals(dyLinear, dyManual, EPSILON);
      
      double ddyLinear = linear.getDerivative(2, x);
      double ddyManual = 0.0;
      assertEquals(ddyLinear, ddyManual, EPSILON); 
   }
   
   @Test
   public void testLinearDerivativePointAutomated()
   {
      //linear polynomial: y(x) = a0 + a1*x
      YoRegistry registry = new YoRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoPolynomial linear = new YoPolynomial(namePrefix + "Linear", numberOfCoefficients, registry);
      
      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;
      
      linear.setLinear(x0, xf, y0, yf);
      
      double x = 2.0/3.0 * (xf - x0);

      compareDerivativesPoint(linear, x);
   }
   
   @Test
   public void testCubicDerivativePointAutomated()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoRegistry registry = new YoRegistry(namePrefix);
      int numberOfCoefficients = 4;
      YoPolynomial cubic = new YoPolynomial(namePrefix + "Cubic", numberOfCoefficients, registry);
      
      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;
      double dy0 = -0.5, dyf = 2.0;
            
      cubic.setCubic(x0, xf, y0, dy0, yf, dyf);
      
      double x = 2.0/3.0 * (xf - x0);

      compareDerivativesPoint(cubic, x);
   }
   
   @Test
   public void testXPowersDerivativeVectorCubic()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoRegistry registry = new YoRegistry(namePrefix);
      int numberOfCoefficients = 4;
      YoPolynomial cubic = new YoPolynomial(namePrefix + "Cubic", numberOfCoefficients, registry);
      
      int numTrials = 9;
      for(int i = 0; i < numTrials; i++)
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
   
   @Test
   public void testDerivativeCoefficients()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoRegistry registry = new YoRegistry(namePrefix);
      int numberOfCoefficients = 8;
      YoPolynomial septic = new YoPolynomial(namePrefix + "Septic", numberOfCoefficients, registry);
      
      double x0 = 1.0, x1 = 1.2, x2 = 1.9, xf = 2.0;
      double y0 = 0.5, y1 = -0.75, y2 = 1.3, yf = 1.5;
      double dy0 = -0.5, dy1 = 0.5, dy2 = 1.0, dyf = 2.0;
            
      septic.setSeptic(x0, x1, x2, xf, y0, dy0, y1, dy1, dy2, dy2, yf, dyf);
      
      int order3Exponent1Func = septic.getDerivativeCoefficient(3, 1);
      int order3Exponent1Hand = 0;
      assertEquals(order3Exponent1Func, order3Exponent1Hand, EPSILON);
      
      int order6Exponent7Func = septic.getDerivativeCoefficient(6, 7);
      int order6Exponent7Hand = 5040;
      assertEquals(order6Exponent7Func, order6Exponent7Hand, EPSILON);
      
      int order0Exponent5Func = septic.getDerivativeCoefficient(0, 5);
      int order0Exponent5Hand = 1;
      assertEquals(order0Exponent5Func, order0Exponent5Hand, EPSILON);
      
      int order3Exponent4Func = septic.getDerivativeCoefficient(3, 4);
      int order3Exponent4Hand = 24;
      assertEquals(order3Exponent4Func, order3Exponent4Hand, EPSILON);
      
      int order5Exponent2Func = septic.getDerivativeCoefficient(5, 2);
      int order5Exponent2Hand = 0;
      assertEquals(order5Exponent2Func, order5Exponent2Hand, EPSILON);
      
      int order1Exponent5Func = septic.getDerivativeCoefficient(1, 5);
      int order1Exponent5Hand = 5;
      assertEquals(order1Exponent5Func, order1Exponent5Hand, EPSILON);
      
      int order11Exponent1Func = septic.getDerivativeCoefficient(11, 1);
      int order11Exponent1Hand = 0;
      assertEquals(order11Exponent1Func, order11Exponent1Hand, EPSILON);
      
      int order13Exponent8Func = septic.getDerivativeCoefficient(13, 8);
      int order13Exponent8Hand = 0;
      assertEquals(order13Exponent8Func, order13Exponent8Hand, EPSILON);
   }
   
   @Test
   public void testDerivativeVersionsCubic()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoRegistry registry = new YoRegistry(namePrefix);
      int numberOfCoefficients = 4;
      YoPolynomial cubic = new YoPolynomial(namePrefix + "Cubic", numberOfCoefficients, registry);
      
      int numTrials = 9;
      for(int i = 0; i < numTrials; i++)
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
   
   
   public void compareDerivativesPoint(YoPolynomial polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for(int i = 0; i < coefficients.length + 3; i++)
      {
         double generalizedDYPoly = polynomial.getDerivative(i, x);
         
         double generalizedDYHand = 0.0;
         if(i < coefficients.length)
         {
            for(int j = i; j < coefficients.length; j++)
            {
               double derivativeCoefficient = polynomial.getDerivativeCoefficient(i, j);
               generalizedDYHand += coefficients[j] * derivativeCoefficient * Math.pow(x, j-i);
            }
         }
         else
         {
            generalizedDYHand = 0.0;
         }
         assertEquals(generalizedDYPoly, generalizedDYHand, EPSILON);
      }
   }
   
   public void compareXPowersDerivativesVector(YoPolynomial polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for(int i = 0; i < coefficients.length + 3; i++)
      {
         DMatrixRMaj generalizedDYPoly = polynomial.getXPowersDerivativeVector(i, x);
         DMatrixRMaj generalizedDYHand = new DMatrixRMaj(generalizedDYPoly.getNumRows(), generalizedDYPoly.getNumCols());
         if(i < coefficients.length)
         {
            for(int j = i; j < coefficients.length; j++)
            {
               double derivativeCoefficient = polynomial.getDerivativeCoefficient(i, j);
               generalizedDYHand.set(j, 0, derivativeCoefficient * Math.pow(x, j - i));        
            }
         }
         for(int k = 0; k < coefficients.length; k++)
         {
            assertEquals(generalizedDYPoly.get(k, 0), generalizedDYHand.get(k, 0), EPSILON);
         }
      }
   }
   
   public void compareDerivativeVersions(YoPolynomial polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for(int i = 0; i < coefficients.length + 3; i++)
      {
         double generalizedDYPolyScalar = polynomial.getDerivative(i, x);
         double generalizedDYHandScalar = 0.0;
         
         DMatrixRMaj generalizedDYPolyVector = polynomial.getXPowersDerivativeVector(i, x);
         for(int j = 0; j < generalizedDYPolyVector.numRows; j++)
         {
            generalizedDYHandScalar += generalizedDYPolyVector.get(j,0) * coefficients[j];
         }
         assertEquals(generalizedDYPolyScalar, generalizedDYHandScalar, EPSILON);
      }
   }

   @Test
   public void testLinearSet()
   {
      YoPolynomial traj = new YoPolynomial("", 2, new YoRegistry("test"));
      assertEquals(0, traj.getNumberOfCoefficients());
      traj.setLinear(1, 2, 3, 5);
      assertEquals(1, traj.getInitialTime(), EPSILON);
      assertEquals(2, traj.getFinalTime(), EPSILON);
      traj.compute(traj.getInitialTime());
      assertEquals(3.0, traj.getValue(), EPSILON);
      traj.compute(traj.getFinalTime());
      assertEquals(5.0, traj.getValue(), EPSILON);
      assertEquals(2, traj.getCoefficient(1), EPSILON);
      assertEquals(1, traj.getCoefficient(0), EPSILON);
   }

   @Test
   public void testSetConstant() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = random.nextInt(10);
         YoPolynomial trajectory = new YoPolynomial("", maxNumberOfCoefficients, new YoRegistry("test"));
         double t0 = random.nextDouble();
         double tf = t0 + random.nextDouble();
         double z = RandomNumbers.nextDouble(random, 1.0);

         if (maxNumberOfCoefficients < 1)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setConstant(t0, tf, z));
            continue;
         }

         trajectory.setConstant(t0, tf, z);

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            assertEquals(z, trajectory.getValue(), EPSILON);
            assertEquals(0, trajectory.getVelocity(), EPSILON);
            assertEquals(0, trajectory.getAcceleration(), EPSILON);
         }
      }
   }

   @Test
   public void testSetLinear() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = random.nextInt(10);
         YoPolynomial trajectory = new YoPolynomial("", maxNumberOfCoefficients, new YoRegistry("test"));
         double t0 = random.nextDouble();
         double tf = t0 + random.nextDouble();
         double z0 = RandomNumbers.nextDouble(random, 1.0);
         double zf = RandomNumbers.nextDouble(random, 1.0);

         if (maxNumberOfCoefficients < 2)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setLinear(t0, tf, z0, zf));
            continue;
         }

         trajectory.setLinear(t0, tf, z0, zf);

         double zDot = (zf - z0) / (tf - t0);
         Polynomial derivative = new Polynomial(1);
         derivative.setConstant(t0, tf, zDot);

         trajectory.compute(t0);
         assertEquals(z0, trajectory.getValue(), EPSILON);
         assertEquals(zDot, trajectory.getVelocity(), EPSILON);
         assertEquals(0, trajectory.getAcceleration(), EPSILON);

         trajectory.compute(tf);
         assertEquals(zf, trajectory.getValue(), EPSILON);
         assertEquals(zDot, trajectory.getVelocity(), EPSILON);
         assertEquals(0, trajectory.getAcceleration(), EPSILON);

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            assertEquals(EuclidCoreTools.interpolate(z0, zf, (t - t0) / (tf - t0)), trajectory.getValue(), EPSILON);
            assertEquals(zDot, trajectory.getVelocity(), EPSILON);
            assertEquals(0, trajectory.getAcceleration(), EPSILON);

            derivative.compute(t);
            assertEquals(derivative.getValue(), trajectory.getVelocity(), EPSILON);
            assertEquals(derivative.getVelocity(), trajectory.getAcceleration(), EPSILON);
         }

         trajectory.setLinear(t0, z0, zDot);

         zDot = (zf - z0) / (tf - t0);
         derivative = new Polynomial(1);
         derivative.setConstant(t0, tf, zDot);

         trajectory.compute(t0);
         assertEquals(z0, trajectory.getValue(), EPSILON);
         assertEquals(zDot, trajectory.getVelocity(), EPSILON);
         assertEquals(0, trajectory.getAcceleration(), EPSILON);

         trajectory.compute(tf);
         assertEquals(zf, trajectory.getValue(), EPSILON);
         assertEquals(zDot, trajectory.getVelocity(), EPSILON);
         assertEquals(0, trajectory.getAcceleration(), EPSILON);

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            assertEquals(EuclidCoreTools.interpolate(z0, zf, (t - t0) / (tf - t0)), trajectory.getValue(), EPSILON);
            assertEquals(zDot, trajectory.getVelocity(), EPSILON);
            assertEquals(0, trajectory.getAcceleration(), EPSILON);

            derivative.compute(t);
            assertEquals(derivative.getValue(), trajectory.getVelocity(), EPSILON);
            assertEquals(derivative.getVelocity(), trajectory.getAcceleration(), EPSILON);
         }
      }
   }

   @Test
   public void testSetQuadratic() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = random.nextInt(10);
         YoPolynomial trajectory = new YoPolynomial("", maxNumberOfCoefficients, new YoRegistry("test"));
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;
         double z0 = RandomNumbers.nextDouble(random, 1.0);
         double zd0 = RandomNumbers.nextDouble(random, 1.0);
         double zf = RandomNumbers.nextDouble(random, 1.0);

         if (maxNumberOfCoefficients < 3)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setQuadratic(t0, tf, z0, zd0, zf));
            continue;
         }

         trajectory.setQuadratic(t0, tf, z0, zd0, zf);

         trajectory.compute(t0);
         assertEquals(z0, trajectory.getValue(), EPSILON);
         assertEquals(zd0, trajectory.getVelocity(), EPSILON);

         trajectory.compute(tf);
         assertEquals(zf, trajectory.getValue(), EPSILON);

         Polynomial derivative = new Polynomial(2);
         derivative.setLinear(t0, tf, zd0, trajectory.getVelocity());

         double dt = 1.0e-8;

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            derivative.compute(t);

            assertEquals(derivative.getValue(), trajectory.getVelocity(), EPSILON);
            assertEquals(derivative.getVelocity(), trajectory.getAcceleration(), EPSILON);

            trajectory.compute(t + dt);
            double nextPosition = trajectory.getValue();
            trajectory.compute(t - dt);
            double lastPosition = trajectory.getValue();
            assertEquals(0.5 * (nextPosition - lastPosition) / dt, trajectory.getVelocity(), 1.0e-6);

         }
      }
   }

   @Test
   public void testSetCubic() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = random.nextInt(10);
         YoPolynomial trajectory = new YoPolynomial("", maxNumberOfCoefficients, new YoRegistry("test"));
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;
         double z0 = RandomNumbers.nextDouble(random, 1.0);
         double zd0 = RandomNumbers.nextDouble(random, 1.0);
         double zf = RandomNumbers.nextDouble(random, 1.0);
         double zdf = RandomNumbers.nextDouble(random, 1.0);

         if (maxNumberOfCoefficients < 4)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setCubic(t0, tf, z0, zd0, zf, zdf));
            continue;
         }

         trajectory.setCubic(t0, tf, z0, zd0, zf, zdf);

         trajectory.compute(t0);
         assertEquals(z0, trajectory.getValue(), EPSILON);
         assertEquals(zd0, trajectory.getVelocity(), EPSILON);

         YoPolynomial derivative = new YoPolynomial("", 3, new YoRegistry("test"));
         derivative.setQuadratic(t0, tf, zd0, trajectory.getAcceleration(), zdf);

         trajectory.compute(tf);
         assertEquals(zf, trajectory.getValue(), EPSILON);

         double dt = 1.0e-8;

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            derivative.compute(t);

            assertEquals(derivative.getValue(), trajectory.getVelocity(), EPSILON);
            assertEquals(derivative.getVelocity(), trajectory.getAcceleration(), EPSILON);

            trajectory.compute(t + dt);
            double nextPosition = trajectory.getValue();
            trajectory.compute(t - dt);
            double lastPosition = trajectory.getValue();
            assertEquals(0.5 * (nextPosition - lastPosition) / dt, trajectory.getVelocity(), 5.0e-6);

         }
      }
   }






   @Test
   public void testQuinticTrajectory()
   {
      Polynomial quinticTrajectoryBase = new Polynomial(-10.0, 10.0, new double[] {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
      YoPolynomial quinticTrajectory = new YoPolynomial("", 6, new YoRegistry("test"));
      quinticTrajectory.set(quinticTrajectoryBase);

      quinticTrajectory.setQuintic(0.0, 1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

      quinticTrajectory.compute(0.0);
      assertEquals(quinticTrajectory.getValue(), 1.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), 2.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      assertEquals(quinticTrajectory.getValue(), 4.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), 5.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);

      quinticTrajectory.setQuintic(-1.0, 1.0, 1.0, -2.0, 3.0, -4.0, -5.0, 6.0);

      quinticTrajectory.compute(-1.0);
      assertEquals(quinticTrajectory.getValue(), 1.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), -2.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      assertEquals(quinticTrajectory.getValue(), -4.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), -5.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);
   }
}