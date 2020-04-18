package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.tools.EuclidCoreTools.square;
import static us.ihmc.robotics.physics.ContactImpulseRandomTools.nextPositiveDefiniteSymmetricMatrix;
import static us.ihmc.robotics.physics.ContactImpulseRandomTools.nextSlippingClosingVelocity;
import static us.ihmc.robotics.physics.ContactImpulseRandomTools.nextSquareFullRank;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeE1;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeE2;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeE3;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeEThetaNumericalDerivative;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeLambda;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeLambdaZ;
import static us.ihmc.robotics.physics.ContactImpulseTools.computePostImpulseVelocity;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeProjectedGradient;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeProjectedGradientInefficient;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeR;
import static us.ihmc.robotics.physics.ContactImpulseTools.computeSlipLambda;
import static us.ihmc.robotics.physics.ContactImpulseTools.cross;
import static us.ihmc.robotics.physics.ContactImpulseTools.invert;
import static us.ihmc.robotics.physics.ContactImpulseTools.isInsideFrictionCone;
import static us.ihmc.robotics.physics.ContactImpulseTools.lineOfSightTest;
import static us.ihmc.robotics.physics.ContactImpulseTools.multQuad;
import static us.ihmc.robotics.physics.ContactImpulseTools.negateMult;
import static us.ihmc.robotics.physics.ContactImpulseTools.polarGradient2;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ContactImpulseToolsTest
{
   private static final int ITERATIONS = 5000;
   private static final double EPSILON = 1.0e-12;
   private static final double PROJ_GRADIENT_EPSILON = 1.0e-9;
   private static final double COST_VS_NAIVE_EPSILON = 1.0e-7;

   @Test
   public void testCross()
   {
      Random random = new Random(36457);

      for (int i = 0; i < ITERATIONS; i++)
      {
         DenseMatrix64F v1 = RandomMatrices.createRandom(3, 1, -10.0, 10.0, random);
         DenseMatrix64F v2 = RandomMatrices.createRandom(3, 1, -10.0, 10.0, random);
         DenseMatrix64F actualCross = cross(v1, v2);

         Vector3D euclidV1 = new Vector3D();
         Vector3D euclidV2 = new Vector3D();
         Vector3D euclidCross = new Vector3D();
         euclidV1.set(v1);
         euclidV2.set(v2);
         euclidCross.cross(euclidV1, euclidV2);
         DenseMatrix64F expectedCross = new DenseMatrix64F(3, 1);
         euclidCross.get(expectedCross);

         assertTrue(MatrixFeatures.isEquals(expectedCross, actualCross, EPSILON));
      }
   }

   @Test
   public void testMultQuad()
   {
      Random random = new Random(46456);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int xSize = random.nextInt(50) + 1;
         DenseMatrix64F x = RandomMatrices.createRandom(xSize, 1, -10.0, 10.0, random);
         DenseMatrix64F H = RandomMatrices.createRandom(xSize, xSize, -10.0, 10.0, random);

         DenseMatrix64F intermediate = new DenseMatrix64F(xSize, 1);
         CommonOps.mult(H, x, intermediate);
         double expectedResult = CommonOps.dot(x, intermediate);
         double actualResult = multQuad(x, H);
         assertEquals(expectedResult, actualResult, EPSILON);
      }
   }

   @Test
   public void testComputeLambdaZ()
   {
      Random random = new Random(46457);

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing against the desired property of lambda_z that is canceling the velocity along the z-axis.
        // Assuming M is full rank
         double r = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cosTheta = Math.cos(theta);
         double sinTheta = Math.sin(theta);
         DenseMatrix64F M_inv = nextSquareFullRank(random);
         DenseMatrix64F c = RandomMatrices.createRandom(3, 1, random);

         double lambda_x = r * cosTheta;
         double lambda_y = r * sinTheta;
         double lambda_z = computeLambdaZ(r, cosTheta, sinTheta, M_inv, c);
         DenseMatrix64F lambda = new DenseMatrix64F(3, 1);
         lambda.set(0, 0, lambda_x);
         lambda.set(1, 0, lambda_y);
         lambda.set(2, 0, lambda_z);
         DenseMatrix64F v = new DenseMatrix64F(c);
         CommonOps.multAdd(M_inv, lambda, v);

         assertEquals(0.0, v.get(2, 0), EPSILON, "Iteration: " + i);
      }
   }

   @Test
   public void testComputeR()
   {
      Random random = new Random(463578);

      for (int i = 0; i < ITERATIONS; i++)
      { // Asserts that the computed results in a lambda that lies on the friction cone surface.
         double mu = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cosTheta = Math.cos(theta);
         double sinTheta = Math.sin(theta);
         DenseMatrix64F M_inv = nextSquareFullRank(random);
         DenseMatrix64F c = RandomMatrices.createRandom(3, 1, random);
         double r = computeR(mu, cosTheta, sinTheta, M_inv, c);
         double lambda_x = Math.abs(r * cosTheta);
         double lambda_y = Math.abs(r * sinTheta);
         double lambda_z = Math.abs(computeLambdaZ(r, cosTheta, sinTheta, M_inv, c));
         double delta = Math.max(1.0, mu * lambda_z) * EPSILON;
         assertEquals(EuclidCoreTools.norm(lambda_x, lambda_y), mu * lambda_z, delta, "Iteration: " + i);
      }
   }

   @Test
   public void testComputeE()
   {
      Random random = new Random(6547);

      for (int i = 0; i < ITERATIONS; i++)
      {
         DenseMatrix64F M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DenseMatrix64F M_inv = invert(M);
         DenseMatrix64F c = RandomMatrices.createRandom(3, 1, -10.0, 10.0, random);
         DenseMatrix64F lambda = RandomMatrices.createRandom(3, 1, -10.0, 10.0, random);

         DenseMatrix64F v = computePostImpulseVelocity(c, M_inv, lambda);

         double E1 = computeE1(v, M);
         double E3 = computeE3(M, M_inv, c, lambda);

         assertEquals(E1, E3, Math.max(1.0, Math.abs(E1)) * EPSILON);
      }
   }

   @Test
   public void testComputeProjectedGradient()
   {
      Random random = new Random(97245);

      for (int i = 0; i < ITERATIONS; i++)
      { // Using the stick solution and adjusting mu such that [lambda_x, lambda_y]^2 = mu^2 * lambda_z^2. The projected gradient should be 0.0 as it is the optimal solution.
         DenseMatrix64F M = nextSquareFullRank(random);
         DenseMatrix64F M_inv = new DenseMatrix64F(3, 3);
         CommonOps.invert(M, M_inv);
         DenseMatrix64F c = RandomMatrices.createRandom(3, 1, random);
         DenseMatrix64F lambda = negateMult(M, c);
         double mu = Math.sqrt(lambda.get(0) * lambda.get(0) + lambda.get(1) * lambda.get(1)) / Math.abs(lambda.get(2));

         double projectedGradient = computeProjectedGradient(mu, M_inv, c, lambda);
         assertEquals(0.0, projectedGradient, PROJ_GRADIENT_EPSILON, "Iteration " + i);
         double cost = computeE1(computePostImpulseVelocity(c, M_inv, lambda), M);
         assertEquals(0.0, cost, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Compare the sign the projected gradient against numerical differentiation of E over theta
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         DenseMatrix64F M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DenseMatrix64F M_inv = invert(M);

         double mu = EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 1.0);
         DenseMatrix64F c = nextSlippingClosingVelocity(random, M_inv, mu);
         double dtheta = 1.0e-6;
         double numericalDerivative = computeEThetaNumericalDerivative(theta, dtheta, mu, M_inv, c);
         DenseMatrix64F lambda_v_0 = negateMult(M, c);
         if (lambda_v_0.get(2) < 0.0)
         {
            i--;
            continue;
         }
         assertFalse(isInsideFrictionCone(mu, lambda_v_0));
         DenseMatrix64F lambda = computeLambda(theta, mu, M_inv, c);
         if (lambda.get(2) < 0.0)
         { // TODO: Identify whether this kind scenario happens with real data.
            i--;
            continue;
         }
         double projectedDerivative = computeProjectedGradient(mu, M_inv, c, lambda);

         assertTrue(numericalDerivative * projectedDerivative > 0.0, "Iteration " + i + ": expected " + numericalDerivative + " was " + projectedDerivative);
      }
   }

   @Test
   public void testLineOfSight()
   {
      Random random = new Random(34563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         DenseMatrix64F M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DenseMatrix64F M_inv = invert(M);

         double mu = EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 1.0);
         DenseMatrix64F c = nextSlippingClosingVelocity(random, M_inv, mu);
         assertTrue(c.get(2) < 0.0);
         DenseMatrix64F lambda_v_0 = negateMult(M, c);
         assertTrue(lambda_v_0.get(2) > 0.0);
         double theta = Math.atan2(lambda_v_0.get(1), lambda_v_0.get(0));
         assertTrue(lineOfSightTest(mu, computeLambda(theta, mu, M_inv, c), lambda_v_0), "Iteration " + i);

         //         double thetaOtherSide = theta + Math.PI;
         //         assertFalse(lineOfSightTest(mu, computeLambda(thetaOtherSide, mu, M_inv, c), lambda_v_0), "Iteration " + i);
      }
   }

   @Test
   public void testNextSlippingClosingVelocity()
   {
      Random random = new Random(4252465);

      for (int i = 0; i < ITERATIONS; i++)
      {
         DenseMatrix64F M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DenseMatrix64F M_inv = invert(M);
         double mu = EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 1.0);
         DenseMatrix64F c = nextSlippingClosingVelocity(random, M_inv, mu);

         DenseMatrix64F lambda_v_0 = negateMult(M, c);
         assertFalse(isInsideFrictionCone(mu, lambda_v_0), "Iteration " + i);
         assertTrue(c.get(2) < 0.0, "Iteration " + i);
      }
   }

   @Test
   public void testComputeSlipLambda()
   {
      Random random = new Random(4353466);
      double beta1 = 0.10;
      double beta2 = 0.95;
      double beta3 = 1.15;
      double gamma = 1.0e-12;

      for (int i = 0; i < ITERATIONS; i++)
      {
         DenseMatrix64F M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DenseMatrix64F M_inv = invert(M);

         double mu = EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 1.0);
         DenseMatrix64F c = nextSlippingClosingVelocity(random, M_inv, mu);
         DenseMatrix64F lambda_v_0 = negateMult(M, c);
         assertFalse(isInsideFrictionCone(mu, lambda_v_0));
         assertTrue(c.get(2) < 0.0);

         double dTheta = 1.0e-2;
         double thetaNaiveOpt = EuclidCoreTools.trimAngleMinusPiToPi(findOptimalTheta(M, M_inv, c, mu, gamma, dTheta, false));
         DenseMatrix64F expectedLambda = computeLambda(thetaNaiveOpt, mu, M_inv, c);
         double expectedCost = computeE2(M_inv, c, expectedLambda);

         DenseMatrix64F actualLambda;
         try
         {
            actualLambda = computeSlipLambda(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c, false);
         }
         catch (IllegalStateException e)
         {
            e.printStackTrace();
            throw new AssertionFailedError("Iteration " + i, e);
         }

         DenseMatrix64F vPlusActual = computePostImpulseVelocity(c, M_inv, actualLambda);
         assertEquals(0.0, vPlusActual.get(2), EPSILON);
         double actualCost = computeE2(M_inv, c, actualLambda);

         while (!EuclidCoreTools.epsilonEquals(expectedCost, actualCost, Math.max(Math.abs(expectedCost), 1.0) * EPSILON))
         {
            dTheta = 0.5 * dTheta;
            System.out.println("Iteration " + i + " dTheta " + dTheta);
            thetaNaiveOpt = EuclidCoreTools.trimAngleMinusPiToPi(findOptimalTheta(M, M_inv, c, mu, gamma, dTheta, false));
            expectedLambda = computeLambda(thetaNaiveOpt, mu, M_inv, c);
            expectedCost = computeE2(M_inv, c, expectedLambda);
         }

         DenseMatrix64F vPlusExpected = computePostImpulseVelocity(c, M_inv, expectedLambda);
         assertTrue(actualLambda.get(0) * vPlusExpected.get(0) + actualLambda.get(1) * vPlusExpected.get(1) < 0.0);

         assertTrue(lineOfSightTest(mu, expectedLambda, lambda_v_0));
         assertTrue(isInsideFrictionCone(mu, actualLambda, Math.max(1.0, CommonOps.elementMaxAbs(actualLambda)) * EPSILON));
         assertTrue(isInsideFrictionCone(mu, expectedLambda, Math.max(1.0, CommonOps.elementMaxAbs(expectedLambda)) * EPSILON));

         assertEquals(expectedCost, actualCost, Math.max(Math.abs(expectedCost), 1.0) * EPSILON, "Iteration " + i);

         boolean areEqual = MatrixFeatures.isEquals(expectedLambda,
                                                    actualLambda,
                                                    Math.max(1.0, CommonOps.elementMaxAbs(expectedLambda)) * COST_VS_NAIVE_EPSILON);
         if (!areEqual)
         {
            System.out.println("iteration: " + i);

            System.out.println("Cost: " + expectedCost + ", " + actualCost);

            double maxError = 0.0;
            DenseMatrix64F output = new DenseMatrix64F(3, 3);

            for (int row = 0; row < 3; row++)
            {
               double error = expectedLambda.get(row, 0) - actualLambda.get(row, 0);

               output.set(row, 0, expectedLambda.get(row, 0));
               output.set(row, 1, actualLambda.get(row, 0));
               output.set(row, 2, error);

               maxError = Math.max(maxError, Math.abs(error));
            }
            output.print(EuclidCoreIOTools.getStringFormat(9, 6));
            System.out.println("Max error: " + maxError);
            System.out.println("c: " + c);

            System.out.println("v+:");
            DenseMatrix64F vPluses = new DenseMatrix64F(3, 2);
            CommonOps.insert(vPlusExpected, vPluses, 0, 0);
            CommonOps.insert(vPlusActual, vPluses, 0, 1);
            System.out.println(vPluses);
         }
         assertTrue(areEqual);
      }
   }

   static double polarGradient(DenseMatrix64F M_inv, double theta, DenseMatrix64F c, double lambda_z, double mu)
   { // Obtained from Tobias Preclick PhD thesis
      double cosTheta = Math.cos(theta);
      double sinTheta = Math.sin(theta);

      double Minv_xx = M_inv.get(0, 0);
      double Minv_yy = M_inv.get(1, 1);
      double Minv_xy = M_inv.get(0, 1);
      double Minv_zy = M_inv.get(2, 1);
      double Minv_zx = M_inv.get(2, 0);
      double c_x = c.get(0);
      double c_y = c.get(1);

      double dE_dTheta = (Minv_xy * cosTheta * cosTheta + (Minv_yy - Minv_xx) * sinTheta * cosTheta - Minv_xy * sinTheta * sinTheta) * square(mu * lambda_z);
      dE_dTheta += ((c_y + Minv_zy * lambda_z) * cosTheta - (c_x + Minv_zx * lambda_z) * sinTheta) * mu * lambda_z;
      return dE_dTheta;
   }

   public static double findOptimalTheta(DenseMatrix64F M, DenseMatrix64F M_inv, DenseMatrix64F c, double mu, double tolerance, double dTheta, boolean verbose)
   {
      double thetaStart = 0.0;
      double thetaSubOpt = thetaStart;
      double costSubOpt = Double.POSITIVE_INFINITY;
      DenseMatrix64F lambdaSubOpt;

      double thetaOpt = thetaSubOpt;
      double costOpt = Double.POSITIVE_INFINITY;

      do
      {
         thetaSubOpt = findNextSubOptimalTheta(thetaSubOpt, M_inv, c, mu, tolerance, dTheta, verbose);
         lambdaSubOpt = computeLambda(thetaSubOpt, mu, M_inv, c);
         costSubOpt = computeE2(M_inv, c, lambdaSubOpt);

         if (costSubOpt < costOpt && lambdaSubOpt.get(2) > 0.0)
         {
            costOpt = costSubOpt;
            thetaOpt = thetaSubOpt;
         }
      }
      while (thetaSubOpt - thetaStart < 2.0 * Math.PI);

      return thetaOpt;
   }

   public static double findNextSubOptimalTheta(double thetaStart, DenseMatrix64F M_inv, DenseMatrix64F c, double mu, double tolerance, double dTheta,
                                                boolean verbose)
   {
      double currentTheta = thetaStart;
      double previousTheta;
      double currentCost = computeE2(M_inv, c, computeLambda(currentTheta, mu, M_inv, c));
      double previousCost;
      double currentGradient = computeEThetaNumericalDerivative(currentTheta, 1.0e-6, mu, M_inv, c);

      do
      { // Going to the next portion where the cost reduce as theta increase.
         previousTheta = currentTheta;
         previousCost = currentCost;
         currentTheta = previousTheta + dTheta;
         currentCost = computeE2(M_inv, c, computeLambda(currentTheta, mu, M_inv, c));
         currentGradient = polarGradient2(M_inv, c, currentTheta, mu);
         if (verbose)
         {
            System.out.println("going up, theta: " + currentTheta + ", cost: " + currentCost + ", grad: "
                  + computeProjectedGradient(mu, M_inv, c, currentTheta));
         }
      }
      while (currentCost - previousCost > 0.0);

      double bisectionTheta = currentTheta;
      double bisectionCost = currentCost;
      double bisectionGradient = currentGradient;

      do
      { // Going to the next point where the cost crosses zero.
         previousTheta = currentTheta;
         previousCost = currentCost;
         currentTheta = previousTheta + dTheta;
         currentCost = computeE2(M_inv, c, computeLambda(currentTheta, mu, M_inv, c));
         currentGradient = polarGradient2(M_inv, c, currentTheta, mu);
         if (verbose)
         {
            System.out.println("going down, theta: " + currentTheta + ", cost: " + currentCost + ", grad: "
                  + computeProjectedGradient(mu, M_inv, c, currentTheta));
         }
      }
      while (currentCost - previousCost < 0.0);

      // Now bisection between previousTheta and currentTheta
      while (true)
      {
         double thetaMid = 0.5 * (bisectionTheta + currentTheta);
         double costMid = computeE2(M_inv, c, computeLambda(thetaMid, mu, M_inv, c));
         double gradientMid = polarGradient2(M_inv, c, thetaMid, mu);

         if (bisectionGradient * gradientMid > 0.0)
         {
            bisectionTheta = thetaMid;
            bisectionCost = costMid;
            bisectionGradient = gradientMid;
         }
         else
         {
            currentTheta = thetaMid;
            currentCost = costMid;
         }

         if (verbose)
         {
            System.out.println("theta [" + bisectionTheta + ", " + currentTheta + "], cost [" + bisectionCost + ", " + currentCost + "]");
         }
         if (Math.abs(bisectionTheta - currentTheta) < tolerance)
            return thetaMid;
      }
   }

   @Test
   public void testDatasets()
   {
      List<Dataset> datasets = datasets();

      for (int i = 0; i < datasets.size(); i++)
      {
         Dataset dataset = datasets.get(i);
         double beta1 = dataset.beta1;
         double beta2 = dataset.beta2;
         double beta3 = dataset.beta3;
         double gamma = dataset.gamma;

         double mu = dataset.mu;
         DenseMatrix64F M_inv = dataset.M_inv;
         assertTrue(MatrixFeatures.isSymmetric(M_inv, EPSILON));
         assertTrue(MatrixFeatures.isPositiveSemidefinite(M_inv));
         DenseMatrix64F M = invert(M_inv);
         DenseMatrix64F c = dataset.c;
         assertTrue(c.get(2) < 0.0);
         DenseMatrix64F lambda_v_0 = negateMult(M, c);
         assertFalse(isInsideFrictionCone(mu, lambda_v_0));

         double dTheta = 1.0e-2;
         double thetaNaiveOpt = EuclidCoreTools.trimAngleMinusPiToPi(findOptimalTheta(M, M_inv, c, mu, gamma, dTheta, false));
         DenseMatrix64F expectedLambda = computeLambda(thetaNaiveOpt, mu, M_inv, c);
         double expectedCost = computeE2(M_inv, c, expectedLambda);

         DenseMatrix64F actualLambda;
         try
         {
            actualLambda = computeSlipLambda(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c, i == 2);
            System.out.println(computeProjectedGradient(mu, M_inv, c, actualLambda));
            System.out.println(computeProjectedGradientInefficient(M_inv, actualLambda, c, mu));
         }
         catch (IllegalStateException e)
         {
            e.printStackTrace();
            throw new AssertionFailedError("Iteration " + i, e);
         }

         DenseMatrix64F vPlusActual = computePostImpulseVelocity(c, M_inv, actualLambda);
         assertEquals(0.0, vPlusActual.get(2), EPSILON, "Iteration " + i);
         assertTrue(lineOfSightTest(mu, actualLambda, lambda_v_0), "Iteration " + i);
         assertTrue(isInsideFrictionCone(mu, actualLambda, Math.max(1.0, CommonOps.elementMaxAbs(actualLambda)) * EPSILON), "Iteration " + i);
         assertTrue(actualLambda.get(0) * vPlusActual.get(0) + actualLambda.get(1) * vPlusActual.get(1) < 0.0, "Iteration " + i);

         double actualCost = computeE2(M_inv, c, actualLambda);

         while (!EuclidCoreTools.epsilonEquals(expectedCost, actualCost, Math.max(Math.abs(expectedCost), 1.0) * EPSILON))
         {
            dTheta = 0.5 * dTheta;
            System.out.println("Iteration " + i + " dTheta " + dTheta);
            thetaNaiveOpt = EuclidCoreTools.trimAngleMinusPiToPi(findOptimalTheta(M, M_inv, c, mu, gamma, dTheta, false));
            expectedLambda = computeLambda(thetaNaiveOpt, mu, M_inv, c);
            expectedCost = computeE2(M_inv, c, expectedLambda);
         }

         DenseMatrix64F vPlusExpected = computePostImpulseVelocity(c, M_inv, expectedLambda);

         assertTrue(lineOfSightTest(mu, expectedLambda, lambda_v_0), "Iteration " + i);
         assertTrue(isInsideFrictionCone(mu, expectedLambda, Math.max(1.0, CommonOps.elementMaxAbs(expectedLambda)) * EPSILON), "Iteration " + i);

         assertEquals(expectedCost, actualCost, Math.max(Math.abs(expectedCost), 1.0) * EPSILON, "Iteration " + i);

         boolean areEqual = MatrixFeatures.isEquals(expectedLambda,
                                                    actualLambda,
                                                    Math.max(1.0, CommonOps.elementMaxAbs(expectedLambda)) * COST_VS_NAIVE_EPSILON);
         if (!areEqual)
         {
            System.out.println("iteration: " + i);

            System.out.println("Cost: " + expectedCost + ", " + actualCost);

            double maxError = 0.0;
            DenseMatrix64F output = new DenseMatrix64F(3, 3);

            for (int row = 0; row < 3; row++)
            {
               double error = expectedLambda.get(row, 0) - actualLambda.get(row, 0);

               output.set(row, 0, expectedLambda.get(row, 0));
               output.set(row, 1, actualLambda.get(row, 0));
               output.set(row, 2, error);

               maxError = Math.max(maxError, Math.abs(error));
            }
            output.print(EuclidCoreIOTools.getStringFormat(9, 6));
            System.out.println("Max error: " + maxError);
            System.out.println("c: " + c);

            System.out.println("v+:");
            DenseMatrix64F vPluses = new DenseMatrix64F(3, 2);
            CommonOps.insert(vPlusExpected, vPluses, 0, 0);
            CommonOps.insert(vPlusActual, vPluses, 0, 1);
            System.out.println(vPluses);
         }
         assertTrue(areEqual);
      }
   }

   private static class Dataset
   {
      private double beta1;
      private double beta2;
      private double beta3;
      private double gamma;
      private double mu;
      private DenseMatrix64F M_inv;
      private DenseMatrix64F c;

      public Dataset(double beta1, double beta2, double beta3, double gamma, double mu, DenseMatrix64F M_inv, DenseMatrix64F c)
      {
         this.beta1 = beta1;
         this.beta2 = beta2;
         this.beta3 = beta3;
         this.gamma = gamma;
         this.mu = mu;
         this.M_inv = M_inv;
         this.c = c;
      }
   }

   private static List<Dataset> datasets()
   {
      List<Dataset> datasets = new ArrayList<>();
      datasets.add(new Dataset(0.10, //1.57,
                               0.95, //1.01,
                               1.05, //0.99,
                               1.0e-12, //1.0E-6,
                               0.7,
                               new DenseMatrix64F(3,
                                                  3,
                                                  true,
                                                  0.12177228584776682,
                                                  -0.006490285501662073,
                                                  -0.0198942344541152,
                                                  -0.0064902855016620966,
                                                  0.03749206992467399,
                                                  -0.020817228276444992,
                                                  -0.019894234454115235,
                                                  -0.020817228276444992,
                                                  0.06819326854414993),
                               new DenseMatrix64F(3, 1, true, -0.9626403389842907, -0.36649553510266114, -1.2699859622739815)));
      datasets.add(new Dataset(0.35,
                               0.95,
                               1.15,
                               1.0E-6,
                               0.7,
                               new DenseMatrix64F(3,
                                                  3,
                                                  true,
                                                  0.5377939246837216,
                                                  0.07285205074934606,
                                                  0.1410180601120504,
                                                  0.07285205074934609,
                                                  0.31156159306905296,
                                                  0.37755617516622175,
                                                  0.14101806011205031,
                                                  0.37755617516622164,
                                                  0.6485524022866964),
                               new DenseMatrix64F(3, 1, true, -0.4009147394246455, 0.48869427996462383, -1.5083682060883388)));
      return datasets;
   }
}
