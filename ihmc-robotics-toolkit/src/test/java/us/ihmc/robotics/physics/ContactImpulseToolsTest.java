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

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
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
         DMatrixRMaj v1 = RandomMatrices_DDRM.rectangle(3, 1, -10.0, 10.0, random);
         DMatrixRMaj v2 = RandomMatrices_DDRM.rectangle(3, 1, -10.0, 10.0, random);
         DMatrixRMaj actualCross = cross(v1, v2);

         Vector3D euclidV1 = new Vector3D();
         Vector3D euclidV2 = new Vector3D();
         Vector3D euclidCross = new Vector3D();
         euclidV1.set(v1);
         euclidV2.set(v2);
         euclidCross.cross(euclidV1, euclidV2);
         DMatrixRMaj expectedCross = new DMatrixRMaj(3, 1);
         euclidCross.get(expectedCross);

         assertTrue(MatrixFeatures_DDRM.isEquals(expectedCross, actualCross, EPSILON));
      }
   }

   @Test
   public void testMultQuad()
   {
      Random random = new Random(46456);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int xSize = random.nextInt(50) + 1;
         DMatrixRMaj x = RandomMatrices_DDRM.rectangle(xSize, 1, -10.0, 10.0, random);
         DMatrixRMaj H = RandomMatrices_DDRM.rectangle(xSize, xSize, -10.0, 10.0, random);

         DMatrixRMaj intermediate = new DMatrixRMaj(xSize, 1);
         CommonOps_DDRM.mult(H, x, intermediate);
         double expectedResult = CommonOps_DDRM.dot(x, intermediate);
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
         DMatrixRMaj M_inv = nextSquareFullRank(random);
         DMatrixRMaj c = RandomMatrices_DDRM.rectangle(3, 1, random);

         double lambda_x = r * cosTheta;
         double lambda_y = r * sinTheta;
         double lambda_z = computeLambdaZ(r, cosTheta, sinTheta, M_inv, c);
         DMatrixRMaj lambda = new DMatrixRMaj(3, 1);
         lambda.set(0, 0, lambda_x);
         lambda.set(1, 0, lambda_y);
         lambda.set(2, 0, lambda_z);
         DMatrixRMaj v = new DMatrixRMaj(c);
         CommonOps_DDRM.multAdd(M_inv, lambda, v);

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
         DMatrixRMaj M_inv = nextSquareFullRank(random);
         DMatrixRMaj c = RandomMatrices_DDRM.rectangle(3, 1, random);
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
         DMatrixRMaj M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DMatrixRMaj M_inv = invert(M);
         DMatrixRMaj c = RandomMatrices_DDRM.rectangle(3, 1, -10.0, 10.0, random);
         DMatrixRMaj lambda = RandomMatrices_DDRM.rectangle(3, 1, -10.0, 10.0, random);

         DMatrixRMaj v = computePostImpulseVelocity(c, M_inv, lambda);

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
         DMatrixRMaj M = nextSquareFullRank(random);
         DMatrixRMaj M_inv = new DMatrixRMaj(3, 3);
         CommonOps_DDRM.invert(M, M_inv);
         DMatrixRMaj c = RandomMatrices_DDRM.rectangle(3, 1, random);
         DMatrixRMaj lambda = negateMult(M, c);
         double mu = Math.sqrt(lambda.get(0) * lambda.get(0) + lambda.get(1) * lambda.get(1)) / Math.abs(lambda.get(2));

         double projectedGradient = computeProjectedGradient(mu, M_inv, c, lambda);
         assertEquals(0.0, projectedGradient, PROJ_GRADIENT_EPSILON, "Iteration " + i);
         double cost = computeE1(computePostImpulseVelocity(c, M_inv, lambda), M);
         assertEquals(0.0, cost, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Compare the sign the projected gradient against numerical differentiation of E over theta
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         DMatrixRMaj M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DMatrixRMaj M_inv = invert(M);

         double mu = EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 1.0);
         DMatrixRMaj c = nextSlippingClosingVelocity(random, M_inv, mu);
         double dtheta = 1.0e-6;
         double numericalDerivative = computeEThetaNumericalDerivative(theta, dtheta, mu, M_inv, c);
         DMatrixRMaj lambda_v_0 = negateMult(M, c);
         if (lambda_v_0.get(2) < 0.0)
         {
            i--;
            continue;
         }
         assertFalse(isInsideFrictionCone(mu, lambda_v_0));
         DMatrixRMaj lambda = computeLambda(theta, mu, M_inv, c);
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
         DMatrixRMaj M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DMatrixRMaj M_inv = invert(M);

         double mu = EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 1.0);
         DMatrixRMaj c = nextSlippingClosingVelocity(random, M_inv, mu);
         assertTrue(c.get(2) < 0.0);
         DMatrixRMaj lambda_v_0 = negateMult(M, c);
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
         DMatrixRMaj M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DMatrixRMaj M_inv = invert(M);
         double mu = EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 1.0);
         DMatrixRMaj c = nextSlippingClosingVelocity(random, M_inv, mu);

         DMatrixRMaj lambda_v_0 = negateMult(M, c);
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
         DMatrixRMaj M = nextPositiveDefiniteSymmetricMatrix(random, 1.0e-3, 10.0);
         DMatrixRMaj M_inv = invert(M);

         double mu = EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 1.0);
         DMatrixRMaj c = nextSlippingClosingVelocity(random, M_inv, mu);
         DMatrixRMaj lambda_v_0 = negateMult(M, c);
         assertFalse(isInsideFrictionCone(mu, lambda_v_0));
         assertTrue(c.get(2) < 0.0);

         double dTheta = 1.0e-2;
         double thetaNaiveOpt = EuclidCoreTools.trimAngleMinusPiToPi(findOptimalTheta(M, M_inv, c, mu, gamma, dTheta, false));
         DMatrixRMaj expectedLambda = computeLambda(thetaNaiveOpt, mu, M_inv, c);
         double expectedCost = computeE2(M_inv, c, expectedLambda);

         DMatrixRMaj actualLambda;
         try
         {
            actualLambda = computeSlipLambda(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c, false);
         }
         catch (IllegalStateException e)
         {
            e.printStackTrace();
            throw new AssertionFailedError("Iteration " + i, e);
         }

         DMatrixRMaj vPlusActual = computePostImpulseVelocity(c, M_inv, actualLambda);
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

         DMatrixRMaj vPlusExpected = computePostImpulseVelocity(c, M_inv, expectedLambda);
         assertTrue(actualLambda.get(0) * vPlusExpected.get(0) + actualLambda.get(1) * vPlusExpected.get(1) < 0.0);

         assertTrue(lineOfSightTest(mu, expectedLambda, lambda_v_0));
         assertTrue(isInsideFrictionCone(mu, actualLambda, Math.max(1.0, CommonOps_DDRM.elementMaxAbs(actualLambda)) * EPSILON));
         assertTrue(isInsideFrictionCone(mu, expectedLambda, Math.max(1.0, CommonOps_DDRM.elementMaxAbs(expectedLambda)) * EPSILON));

         assertEquals(expectedCost, actualCost, Math.max(Math.abs(expectedCost), 1.0) * EPSILON, "Iteration " + i);

         boolean areEqual = MatrixFeatures_DDRM.isEquals(expectedLambda,
                                                         actualLambda,
                                                         Math.max(1.0, CommonOps_DDRM.elementMaxAbs(expectedLambda)) * COST_VS_NAIVE_EPSILON);
         if (!areEqual)
         {
            System.out.println("iteration: " + i);

            System.out.println("Cost: " + expectedCost + ", " + actualCost);

            double maxError = 0.0;
            DMatrixRMaj output = new DMatrixRMaj(3, 3);

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
            DMatrixRMaj vPluses = new DMatrixRMaj(3, 2);
            CommonOps_DDRM.insert(vPlusExpected, vPluses, 0, 0);
            CommonOps_DDRM.insert(vPlusActual, vPluses, 0, 1);
            System.out.println(vPluses);
         }
         assertTrue(areEqual);
      }
   }

   static double polarGradient(DMatrixRMaj M_inv, double theta, DMatrixRMaj c, double lambda_z, double mu)
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

   public static double polarGradient2(DMatrixRMaj M_inv, DMatrixRMaj c, double theta, double mu)
   { // Obtained by directly evaluating dE/dTheta
      double c_x = c.get(0);
      double c_y = c.get(1);
      double c_z = c.get(2);

      double cosTheta = Math.cos(theta);
      double sinTheta = Math.sin(theta);

      double Mxx = M_inv.get(0, 0);
      double Mxy = M_inv.get(0, 1);
      double Myy = M_inv.get(1, 1);
      double Mzx = M_inv.get(2, 0);
      double Mzy = M_inv.get(2, 1);
      double Mzz = M_inv.get(2, 2);

      double Mtheta = Mzx * cosTheta + Mzy * sinTheta;

      return -c_z * mu
            * (((-Mzy * c_x + Mzx * c_y) * Mtheta + (Mzy * (Mxx * cosTheta + Mxy * sinTheta) - Mzx * (Mxy * cosTheta + Myy * sinTheta)) * c_z) * mu * mu
                  + Mzz * (-(sinTheta * Mtheta + Mzy) * c_x + (cosTheta * Mtheta + Mzx) * c_y
                        + (cosTheta * sinTheta * Mxx - Mxy + 2 * sinTheta * sinTheta * Mxy - sinTheta * cosTheta * Myy) * c_z) * mu
                  + Mzz * ((-sinTheta * c_x + cosTheta * c_y) * Mzz + c_z * (Mzx * sinTheta - Mzy * cosTheta)))
            / ContactImpulseTools.cube(Mzz + Mtheta * mu);
   }

   public static double findOptimalTheta(DMatrixRMaj M, DMatrixRMaj M_inv, DMatrixRMaj c, double mu, double tolerance, double dTheta, boolean verbose)
   {
      double thetaStart = 0.0;
      double thetaSubOpt = thetaStart;
      double costSubOpt = Double.POSITIVE_INFINITY;
      DMatrixRMaj lambdaSubOpt;

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

   public static double findNextSubOptimalTheta(double thetaStart, DMatrixRMaj M_inv, DMatrixRMaj c, double mu, double tolerance, double dTheta,
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
         DMatrixRMaj M_inv = dataset.M_inv;
         assertTrue(MatrixFeatures_DDRM.isSymmetric(M_inv, EPSILON));
         assertTrue(MatrixFeatures_DDRM.isPositiveSemidefinite(M_inv));
         DMatrixRMaj M = invert(M_inv);
         DMatrixRMaj c = dataset.c;
         assertTrue(c.get(2) < 0.0);
         DMatrixRMaj lambda_v_0 = negateMult(M, c);
         assertFalse(isInsideFrictionCone(mu, lambda_v_0));

         double dTheta = 1.0e-2;
         double thetaNaiveOpt = EuclidCoreTools.trimAngleMinusPiToPi(findOptimalTheta(M, M_inv, c, mu, gamma, dTheta, false));
         DMatrixRMaj expectedLambda = computeLambda(thetaNaiveOpt, mu, M_inv, c);
         double expectedCost = computeE2(M_inv, c, expectedLambda);

         DMatrixRMaj actualLambda;
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

         DMatrixRMaj vPlusActual = computePostImpulseVelocity(c, M_inv, actualLambda);
         assertEquals(0.0, vPlusActual.get(2), EPSILON, "Iteration " + i);
         assertTrue(lineOfSightTest(mu, actualLambda, lambda_v_0), "Iteration " + i);
         assertTrue(isInsideFrictionCone(mu, actualLambda, Math.max(1.0, CommonOps_DDRM.elementMaxAbs(actualLambda)) * EPSILON), "Iteration " + i);
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

         DMatrixRMaj vPlusExpected = computePostImpulseVelocity(c, M_inv, expectedLambda);

         assertTrue(lineOfSightTest(mu, expectedLambda, lambda_v_0), "Iteration " + i);
         assertTrue(isInsideFrictionCone(mu, expectedLambda, Math.max(1.0, CommonOps_DDRM.elementMaxAbs(expectedLambda)) * EPSILON), "Iteration " + i);

         assertEquals(expectedCost, actualCost, Math.max(Math.abs(expectedCost), 1.0) * EPSILON, "Iteration " + i);

         boolean areEqual = MatrixFeatures_DDRM.isEquals(expectedLambda,
                                                         actualLambda,
                                                         Math.max(1.0, CommonOps_DDRM.elementMaxAbs(expectedLambda)) * COST_VS_NAIVE_EPSILON);
         if (!areEqual)
         {
            System.out.println("iteration: " + i);

            System.out.println("Cost: " + expectedCost + ", " + actualCost);

            double maxError = 0.0;
            DMatrixRMaj output = new DMatrixRMaj(3, 3);

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
            DMatrixRMaj vPluses = new DMatrixRMaj(3, 2);
            CommonOps_DDRM.insert(vPlusExpected, vPluses, 0, 0);
            CommonOps_DDRM.insert(vPlusActual, vPluses, 0, 1);
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
      private DMatrixRMaj M_inv;
      private DMatrixRMaj c;

      public Dataset(double beta1, double beta2, double beta3, double gamma, double mu, DMatrixRMaj M_inv, DMatrixRMaj c)
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
                               new DMatrixRMaj(3,
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
                               new DMatrixRMaj(3, 1, true, -0.9626403389842907, -0.36649553510266114, -1.2699859622739815)));
      datasets.add(new Dataset(0.35,
                               0.95,
                               1.15,
                               1.0E-6,
                               0.7,
                               new DMatrixRMaj(3,
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
                               new DMatrixRMaj(3, 1, true, -0.4009147394246455, 0.48869427996462383, -1.5083682060883388)));
      return datasets;
   }
}
