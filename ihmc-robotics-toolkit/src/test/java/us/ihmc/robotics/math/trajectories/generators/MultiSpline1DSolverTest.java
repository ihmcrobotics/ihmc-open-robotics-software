package us.ihmc.robotics.math.trajectories.generators;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.robotics.math.trajectories.generators.MultiSpline1DSolver.defaultCoefficients;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.robotics.linearDynamicSystems.MatlabChart;
import us.ihmc.robotics.math.functionGenerator.BaseFunctionGenerator;

public class MultiSpline1DSolverTest
{
   @Test
   public void testAddPositionObjective()
   {
      Random random = new Random(1231);

      for (int i = 0; i < 1000; i++)
      {
         double t = random.nextDouble();
         double xd = random.nextDouble();
         double weight = random.nextDouble();
         int numberOfCoefficients = random.nextInt(10) + 1;

         DMatrixRMaj tMatrix = new DMatrixRMaj(numberOfCoefficients, 1);
         { // Computing the tDot matrix
            double tPow = 1.0;
            int row = numberOfCoefficients - 1;

            for (int coeff = 0; coeff < numberOfCoefficients; coeff++)
            {
               tMatrix.set(row--, 0, tPow);
               tPow *= t;
            }
         }

         DMatrixRMaj H_expected = new DMatrixRMaj(numberOfCoefficients, numberOfCoefficients);
         DMatrixRMaj f_expected = new DMatrixRMaj(numberOfCoefficients, 1);
         CommonOps_DDRM.multOuter(tMatrix, H_expected);
         CommonOps_DDRM.scale(weight, H_expected);
         f_expected.set(tMatrix);
         CommonOps_DDRM.scale(-xd * weight, f_expected);

         DMatrixRMaj H_actual = new DMatrixRMaj(numberOfCoefficients, numberOfCoefficients);
         DMatrixRMaj f_actual = new DMatrixRMaj(numberOfCoefficients, 1);
         MultiSpline1DSolver.addPositionObjective(t, xd, weight, numberOfCoefficients, 0, 0, H_actual, f_actual);

         MecanoTestTools.assertDMatrixEquals(H_expected, H_actual, 1.0e-12);
         MecanoTestTools.assertDMatrixEquals(f_expected, f_actual, 1.0e-12);
      }
   }

   @Test
   public void testAddVelocityObjective()
   {
      Random random = new Random(1231);

      for (int i = 0; i < 1000; i++)
      {
         double t = random.nextDouble();
         double xd = random.nextDouble();
         double weight = random.nextDouble();
         int numberOfCoefficients = random.nextInt(10) + 1;

         DMatrixRMaj tDotMatrix = new DMatrixRMaj(numberOfCoefficients, 1);
         { // Computing the tDot matrix
            double tPow = 1.0;
            int row = numberOfCoefficients - 1;

            tDotMatrix.set(row--, 0, 0);

            for (int coeff = 1; coeff < numberOfCoefficients; coeff++)
            {
               tDotMatrix.set(row--, 0, coeff * tPow);
               tPow *= t;
            }
         }

         DMatrixRMaj H_expected = new DMatrixRMaj(numberOfCoefficients, numberOfCoefficients);
         DMatrixRMaj f_expected = new DMatrixRMaj(numberOfCoefficients, 1);
         CommonOps_DDRM.multOuter(tDotMatrix, H_expected);
         CommonOps_DDRM.scale(weight, H_expected);
         f_expected.set(tDotMatrix);
         CommonOps_DDRM.scale(-xd * weight, f_expected);

         DMatrixRMaj H_actual = new DMatrixRMaj(numberOfCoefficients, numberOfCoefficients);
         DMatrixRMaj f_actual = new DMatrixRMaj(numberOfCoefficients, 1);
         MultiSpline1DSolver.addVelocityObjective(t, xd, weight, numberOfCoefficients, 0, 0, H_actual, f_actual);

         MecanoTestTools.assertDMatrixEquals(H_expected, H_actual, 1.0e-12);
         MecanoTestTools.assertDMatrixEquals(f_expected, f_actual, 1.0e-12);
      }
   }

   @Test
   public void testAddAccelerationObjective()
   {
      Random random = new Random(1231);

      for (int i = 0; i < 1000; i++)
      {
         double t = random.nextDouble();
         double xd = random.nextDouble();
         double weight = random.nextDouble();
         int numberOfCoefficients = random.nextInt(10) + 1;

         DMatrixRMaj tDDotMatrix = new DMatrixRMaj(numberOfCoefficients, 1);
         { // Computing the tDot matrix
            int row = numberOfCoefficients - 1;

            tDDotMatrix.set(row--, 0, 0);
            if (row >= 0)
            {
               tDDotMatrix.set(row--, 0, 0);
               double tPow = 1.0;

               for (int coeff = 2; coeff < numberOfCoefficients; coeff++)
               {
                  tDDotMatrix.set(row--, 0, coeff * (coeff - 1.0) * tPow);
                  tPow *= t;
               }
            }
         }

         DMatrixRMaj H_expected = new DMatrixRMaj(numberOfCoefficients, numberOfCoefficients);
         DMatrixRMaj f_expected = new DMatrixRMaj(numberOfCoefficients, 1);
         CommonOps_DDRM.multOuter(tDDotMatrix, H_expected);
         CommonOps_DDRM.scale(weight, H_expected);
         f_expected.set(tDDotMatrix);
         CommonOps_DDRM.scale(-xd * weight, f_expected);

         DMatrixRMaj H_actual = new DMatrixRMaj(numberOfCoefficients, numberOfCoefficients);
         DMatrixRMaj f_actual = new DMatrixRMaj(numberOfCoefficients, 1);
         MultiSpline1DSolver.addAccelerationObjective(t, xd, weight, numberOfCoefficients, 0, 0, H_actual, f_actual);

         MecanoTestTools.assertDMatrixEquals(H_expected, H_actual, 1.0e-11);
         MecanoTestTools.assertDMatrixEquals(f_expected, f_actual, 1.0e-11);
      }
   }

   @Test
   public void testEndpointsObjective()
   {
      // Test the quality of the endpoints when setup as objectives.
      double t0 = 0.0;
      double t1 = 0.5;
      double t2 = 1.0;

      double x0 = 0.0;
      double x1 = 1.0;
      double x2 = 0.0;

      double xd0 = 0.0;
      double xd2 = 0.0;

      MultiSpline1DSolver solver = new MultiSpline1DSolver();
      solver.addWaypoint(t0, x0, xd0);
      solver.addWaypointPosition(t1, x1);
      solver.addWaypoint(t2, x2, xd2);
      solver.solve();

      double expected_xErr0 = Math.abs(x0 - solver.computePosition(t0));
      double expected_xErr2 = Math.abs(x2 - solver.computePosition(t2));
      double expected_xdErr0 = Math.abs(xd0 - solver.computeVelocity(t0));
      double expected_xdErr2 = Math.abs(xd2 - solver.computeVelocity(t2));
      //      System.out.println("xErr0: " + expected_xErr0 + ", xErr2: " + expected_xErr2 + ", xdErr0: " + expected_xdErr0 + ", xdErr2: " + expected_xdErr2);
      //      plot(solver, null, false);

      solver.getWaypoint(0).setVelocityWeight(1.0e6);
      solver.solve();

      double actual_xErr0 = Math.abs(x0 - solver.computePosition(t0));
      double actual_xErr2 = Math.abs(x2 - solver.computePosition(t2));
      double actual_xdErr0 = Math.abs(xd0 - solver.computeVelocity(t0));
      double actual_xdErr2 = Math.abs(xd2 - solver.computeVelocity(t2));
      //      System.out.println("xErr0: " + actual_xErr0 + ", xErr2: " + actual_xErr2 + ", xdErr0: " + actual_xdErr0 + ", xdErr2: " + actual_xdErr2);
      //      plot(solver, null, false);
      assertEquals(expected_xErr0, actual_xErr0, 1.0e-12);
      assertEquals(expected_xErr2, actual_xErr2, 1.0e-12);
      assertEquals(expected_xdErr0, actual_xdErr0, 1.0e-4);
      assertEquals(expected_xdErr2, actual_xdErr2, 1.0e-12);

      solver.getWaypoint(0).setVelocityWeight(Double.POSITIVE_INFINITY);
      solver.getWaypoint(2).setVelocityWeight(1.0e6);
      solver.solve();

      actual_xErr0 = Math.abs(x0 - solver.computePosition(t0));
      actual_xErr2 = Math.abs(x2 - solver.computePosition(t2));
      actual_xdErr0 = Math.abs(xd0 - solver.computeVelocity(t0));
      actual_xdErr2 = Math.abs(xd2 - solver.computeVelocity(t2));
      //      System.out.println("xErr0: " + actual_xErr0 + ", xErr2: " + actual_xErr2 + ", xdErr0: " + actual_xdErr0 + ", xdErr2: " + actual_xdErr2);
      //      plot(solver, null, false);
      assertEquals(expected_xErr0, actual_xErr0, 1.0e-12);
      assertEquals(expected_xErr2, actual_xErr2, 1.0e-12);
      assertEquals(expected_xdErr0, actual_xdErr0, 1.0e-12);
      assertEquals(expected_xdErr2, actual_xdErr2, 1.0e-4);

      // Moving to testing position objectives
      solver.getWaypoint(2).setVelocityWeight(Double.POSITIVE_INFINITY);
      solver.getWaypoint(0).setPositionWeight(1.0e6);
      solver.solve();

      actual_xErr0 = Math.abs(x0 - solver.computePosition(t0));
      actual_xErr2 = Math.abs(x2 - solver.computePosition(t2));
      actual_xdErr0 = Math.abs(xd0 - solver.computeVelocity(t0));
      actual_xdErr2 = Math.abs(xd2 - solver.computeVelocity(t2));
      //      System.out.println("xErr0: " + actual_xErr0 + ", xErr2: " + actual_xErr2 + ", xdErr0: " + actual_xdErr0 + ", xdErr2: " + actual_xdErr2);
      //      plot(solver, null, false);
      assertEquals(expected_xErr0, actual_xErr0, 1.0e-4);
      assertEquals(expected_xErr2, actual_xErr2, 1.0e-12);
      assertEquals(expected_xdErr0, actual_xdErr0, 1.0e-12);
      assertEquals(expected_xdErr2, actual_xdErr2, 1.0e-12);

      solver.getWaypoint(0).setPositionWeight(Double.POSITIVE_INFINITY);
      solver.getWaypoint(2).setPositionWeight(1.0e6);
      solver.solve();

      actual_xErr0 = Math.abs(x0 - solver.computePosition(t0));
      actual_xErr2 = Math.abs(x2 - solver.computePosition(t2));
      actual_xdErr0 = Math.abs(xd0 - solver.computeVelocity(t0));
      actual_xdErr2 = Math.abs(xd2 - solver.computeVelocity(t2));
      //      System.out.println("xErr0: " + actual_xErr0 + ", xErr2: " + actual_xErr2 + ", xdErr0: " + actual_xdErr0 + ", xdErr2: " + actual_xdErr2);
      //      plot(solver, null, false);
      assertEquals(expected_xErr0, actual_xErr0, 1.0e-12);
      assertEquals(expected_xErr2, actual_xErr2, 1.0e-4);
      assertEquals(expected_xdErr0, actual_xdErr0, 1.0e-12);
      assertEquals(expected_xdErr2, actual_xdErr2, 1.0e-12);
   }

   @Test
   public void testMidpointVelocityControl()
   {
      // Test the quality of the endpoints when setup as objectives.
      double t0 = 0.0;
      double t1 = 0.5;
      double t2 = 1.0;

      double x0 = 0.0;
      double x1 = -1.0;
      double x2 = 0.0;

      double xd0 = 0.0;
      double xd1 = 1.0;
      double xd2 = 0.0;

      MultiSpline1DSolver solver = new MultiSpline1DSolver();
      solver.addWaypoint(t0, x0, xd0);
      solver.addWaypoint(t1, x1, xd1);
      solver.addWaypoint(t2, x2, xd2);
      solver.solve();

      double xErr1 = Math.abs(x1 - solver.computePosition(t1));
      double xdErr1 = Math.abs(xd1 - solver.computeVelocity(t1));

      double dt = 1.0e-5;

      double xErr1Plus = Math.abs(x1 - solver.computePosition(t1 + dt));
      double xdErr1Plus = Math.abs(xd1 - solver.computeVelocity(t1 + dt));
      double xErr1Minus = Math.abs(x1 - solver.computePosition(t1 - dt));
      double xdErr1Minus = Math.abs(xd1 - solver.computeVelocity(t1 - dt));

      //      System.out.println("x1: " + solver.computePosition(t1) + ", xd1: " + solver.computeVelocity(t1));
      //      System.out.println("x1-: " + solver.computePosition(t1 - dt) + ", xd1-: " + solver.computeVelocity(t1 - dt));
      //      System.out.println("x1+: " + solver.computePosition(t1 + dt) + ", xd1+: " + solver.computeVelocity(t1 + dt));
      //      plot(solver, null, false);

      assertEquals(xErr1, xErr1Plus, 2.0e-5);
      assertEquals(xErr1, xErr1Minus, 2.0e-5);
      assertEquals(xdErr1, xdErr1Plus, 1.0e-3);
      assertEquals(xdErr1, xdErr1Minus, 1.0e-3);

      solver.getWaypoint(1).setVelocityWeight(1.0e6);
      solver.solve();

      xErr1Plus = Math.abs(x1 - solver.computePosition(t1 + dt));
      xdErr1Plus = Math.abs(xd1 - solver.computeVelocity(t1 + dt));
      xErr1Minus = Math.abs(x1 - solver.computePosition(t1 - dt));
      xdErr1Minus = Math.abs(xd1 - solver.computeVelocity(t1 - dt));

      //      System.out.println("x1: " + solver.computePosition(t1) + ", xd1: " + solver.computeVelocity(t1));
      //      System.out.println("x1-: " + solver.computePosition(t1 - dt) + ", xd1-: " + solver.computeVelocity(t1 - dt));
      //      System.out.println("x1+: " + solver.computePosition(t1 + dt) + ", xd1+: " + solver.computeVelocity(t1 + dt));
      //      plot(solver, null, false);

      solver.getWaypoint(1).setPositionWeight(1.0e6);
      solver.getWaypoint(1).setVelocityWeight(Double.POSITIVE_INFINITY);
      solver.solve();

      xErr1Plus = Math.abs(x1 - solver.computePosition(t1 + dt));
      xdErr1Plus = Math.abs(xd1 - solver.computeVelocity(t1 + dt));
      xErr1Minus = Math.abs(x1 - solver.computePosition(t1 - dt));
      xdErr1Minus = Math.abs(xd1 - solver.computeVelocity(t1 - dt));

      //      System.out.println("x1: " + solver.computePosition(t1) + ", xd1: " + solver.computeVelocity(t1));
      //      System.out.println("x1-: " + solver.computePosition(t1 - dt) + ", xd1-: " + solver.computeVelocity(t1 - dt));
      //      System.out.println("x1+: " + solver.computePosition(t1 + dt) + ", xd1+: " + solver.computeVelocity(t1 + dt));
      //      plot(solver, null, true);
   }

   @Test
   public void testMultiOrders()
   {
      // Test the quality of the endpoints when setup as objectives.
      double t0 = 0.0;
      double t1 = 0.5;
      double t2 = 1.0;

      double x0 = 0.0;
      double x1 = 1.0;
      double x2 = 0.0;

      double xd0 = 0.0;
      double xd2 = 0.0;

      MultiSpline1DSolver solver = new MultiSpline1DSolver();
      solver.addWaypoint(t0, x0, xd0);
      solver.addWaypointPosition(t1, x1);
      solver.addWaypoint(t2, x2, xd2);
      solver.getSplineSegment(0).setNumberOfCoefficients(3);
      solver.solve();

      double xErr0 = Math.abs(x0 - solver.computePosition(t0));
      double xErr1 = Math.abs(x1 - solver.computePosition(t1));
      double xErr2 = Math.abs(x2 - solver.computePosition(t2));
      double xdErr0 = Math.abs(xd0 - solver.computeVelocity(t0));
      double xdErr2 = Math.abs(xd2 - solver.computeVelocity(t2));
      //      System.out.println("xErr0: " + xErr0 + ", xErr1: " + xErr1 + ", xErr2: " + xErr2 + ", xdErr0: " + xdErr0 + ", xdErr2: " + xdErr2);
      //      plot(solver, null, true);

      assertEquals(0.0, xErr0, 1.0e-12);
      assertEquals(0.0, xErr1, 1.0e-12);
      assertEquals(0.0, xErr2, 1.0e-12);
      assertEquals(0.0, xdErr0, 1.0e-12);
      assertEquals(0.0, xdErr2, 1.0e-12);
   }

   @Test
   public void testAgainstSinewave()
   {
      boolean verbose = false;
      Random random = new Random(34243);

      Function function = new SineFunction(random);

      MultiSpline1DSolver solver = nextFunctionBasedMultiSpline(random, function);

      for (int i = 1; i < solver.getNumberOfWaypoints() - 1; i++)
      {
         solver.getWaypoint(i).setVelocityWeight(0.0); // Not constraining the velocity
      }

      solver.solve();
      //      plot(solver, function);

      int numTicks = 5000;
      double t0 = solver.getFirstWaypoint().getTime();
      double tf = solver.getLastWaypoint().getTime();

      double xErrAvg = 0.0;
      double xdErrAvg = 0.0;
      double xErrSqAvg = 0.0;
      double xdErrSqAvg = 0.0;

      for (int i = 0; i < numTicks; i++)
      {
         double t = EuclidCoreTools.interpolate(t0, tf, i / (numTicks - 1.0));
         function.compute(t);
         assertEquals(function.getValue(), solver.computePosition(t), 1.0e-4 * Math.max(1.0, Math.abs(function.getValue())));
         assertEquals(function.getValueDot(), solver.computeVelocity(t), 1.0e-3 * Math.max(1.0, Math.abs(function.getValueDot())));

         if (verbose)
         {
            double xErr = Math.abs(function.getValue() - solver.computePosition(t));
            double xdErr = Math.abs(function.getValueDot() - solver.computeVelocity(t));
            xErrAvg += xErr;
            xdErrAvg += xdErr;
            xErrSqAvg += xErr * xErr;
            xdErrSqAvg += xdErr * xdErr;
         }
      }

      if (verbose)
      {
         xErrAvg /= numTicks;
         xdErrAvg /= numTicks;
         xErrSqAvg /= numTicks;
         xdErrSqAvg /= numTicks;

         System.out.println("xErrAvg: " + xErrAvg + ", xdErrAvg: " + xdErrAvg);
         System.out.println("xErrSqAvg: " + xErrSqAvg + ", xdErrSqAvg: " + xdErrSqAvg);
      }

      // Test with explicit velocity constraints
      for (int i = 1; i < solver.getNumberOfWaypoints() - 1; i++)
      {
         solver.getWaypoint(i).setVelocityWeight(Double.POSITIVE_INFINITY);
      }

      solver.solve();
      //      plot(solver, function);

      xErrAvg = 0.0;
      xdErrAvg = 0.0;
      xErrSqAvg = 0.0;
      xdErrSqAvg = 0.0;

      for (int i = 0; i < numTicks; i++)
      {
         double t = EuclidCoreTools.interpolate(t0, tf, i / (numTicks - 1.0));
         function.compute(t);
         assertEquals(function.getValue(), solver.computePosition(t), 1.0e-4 * Math.max(1.0, Math.abs(function.getValue())));
         assertEquals(function.getValueDot(), solver.computeVelocity(t), 1.0e-3 * Math.max(1.0, Math.abs(function.getValueDot())));

         if (verbose)
         {
            double xErr = Math.abs(function.getValue() - solver.computePosition(t));
            double xdErr = Math.abs(function.getValueDot() - solver.computeVelocity(t));
            xErrAvg += xErr;
            xdErrAvg += xdErr;
            xErrSqAvg += xErr * xErr;
            xdErrSqAvg += xdErr * xdErr;
         }
      }

      if (verbose)
      {
         xErrAvg /= numTicks;
         xdErrAvg /= numTicks;
         xErrSqAvg /= numTicks;
         xdErrSqAvg /= numTicks;

         System.out.println("xErrAvg: " + xErrAvg + ", xdErrAvg: " + xdErrAvg);
         System.out.println("xErrSqAvg: " + xErrSqAvg + ", xdErrSqAvg: " + xdErrSqAvg);
      }

      // Test with velocity weight
      for (int i = 1; i < solver.getNumberOfWaypoints() - 1; i++)
      {
         solver.getWaypoint(i).setVelocityWeight(1.0e6);
      }

      solver.solve();
      //      plot(solver, function);

      xErrAvg = 0.0;
      xdErrAvg = 0.0;
      xErrSqAvg = 0.0;
      xdErrSqAvg = 0.0;

      for (int i = 0; i < numTicks; i++)
      {
         double t = EuclidCoreTools.interpolate(t0, tf, i / (numTicks - 1.0));
         function.compute(t);
         assertEquals(function.getValue(), solver.computePosition(t), 1.0e-4 * Math.max(1.0, Math.abs(function.getValue())));
         assertEquals(function.getValueDot(), solver.computeVelocity(t), 1.0e-3 * Math.max(1.0, Math.abs(function.getValueDot())));

         if (verbose)
         {
            double xErr = Math.abs(function.getValue() - solver.computePosition(t));
            double xdErr = Math.abs(function.getValueDot() - solver.computeVelocity(t));
            xErrAvg += xErr;
            xdErrAvg += xdErr;
            xErrSqAvg += xErr * xErr;
            xdErrSqAvg += xdErr * xdErr;
         }
      }

      if (verbose)
      {
         xErrAvg /= numTicks;
         xdErrAvg /= numTicks;
         xErrSqAvg /= numTicks;
         xdErrSqAvg /= numTicks;

         System.out.println("xErrAvg: " + xErrAvg + ", xdErrAvg: " + xdErrAvg);
         System.out.println("xErrSqAvg: " + xErrSqAvg + ", xdErrSqAvg: " + xdErrSqAvg);
      }
   }

   // Test from the MultiSpline1DSolverTest
   @Test
   public void testAccelerationIntegrationResult()
   {
      MultiSpline1DSolver solver = new MultiSpline1DSolver();
      double[] times = {0.0, 0.25, 0.75, 1.0};
      solver.addWaypoint(times[0], 0, 1);
      solver.addWaypointPosition(times[1], 2);
      solver.addWaypointPosition(times[2], -2);
      solver.addWaypoint(times[3], 0, 1);
      double costUsingNative = solver.solveAndComputeCost();

      DMatrixRMaj solution = solver.getSolution();
      int numberOfSplines = solution.getNumRows() / defaultCoefficients;
      double expectedAccelerationIntegrated = 0.0;

      for (int i = 0; i < numberOfSplines; i++)
      {
         int offset = i * MultiSpline1DSolver.defaultCoefficients;
         double c0 = solution.get(offset + 0);
         double c1 = solution.get(offset + 1);

         double t0 = times[i];
         double tf = times[i + 1];

         // This is the integration of the acceleration squared.
         // The acceleration function is: xDDot = 6 c0 t + 2 c1
         // The squared acceleration function is: xDDot^2 = 12 c0^2 t^2 + 24 c0 c1 t + 4 c1^2
         // The integrated squared acceleration function from t0 to tf is: 12 c0^2 (tf^3 - t0^3) + 12 c0 c1 (tf^2 - t0^2) + 4 c1^2 (tf - t0)
         expectedAccelerationIntegrated += (12.0 * c0 * c0 * (tf * tf * tf - t0 * t0 * t0) + 12.0 * c0 * c1 * (tf * tf - t0 * t0) + 4.0 * c1 * c1 * (tf - t0));
      }

      double dt = 0.00001;
      double accelerationNumericallyIntegrated = 0.0;

      for (double t = dt; t <= 1.0; t += dt)
      {
         double accCurr = solver.computeAcceleration(t);
         accelerationNumericallyIntegrated += MathTools.square(Math.abs(accCurr)) * dt;
      }

      expectedAccelerationIntegrated *= 0.5;
      accelerationNumericallyIntegrated *= 0.5;

      assertEquals(expectedAccelerationIntegrated, costUsingNative, Math.max(expectedAccelerationIntegrated, 1.0) * 1.0e-12);
      assertEquals(accelerationNumericallyIntegrated, costUsingNative, Math.max(accelerationNumericallyIntegrated, 1.0) * 1.0e-7);
   }

   @Test
   public void testGetWaypointVelocityFromSolution()
   {
      Random random = new Random(45435);
      MultiSpline1DSolver solver = new MultiSpline1DSolver();

      for (int i = 0; i < 100; i++)
      {

         int numberOfWaypoints = random.nextInt(10) + 1;

         double startPosition = EuclidCoreRandomTools.nextDouble(random);
         double endPosition = EuclidCoreRandomTools.nextDouble(random);
         double[] waypointPositions = random.doubles(numberOfWaypoints, -0.5, 10.0).toArray();
         double[] waypointTimes = new double[numberOfWaypoints];
         { // Computing the waypoint times
            double[] weights = new double[numberOfWaypoints + 1];
            double sumOfWeights = 0.0;
            for (int j = 0; j < weights.length; j++)
            {
               weights[j] = random.nextDouble();
               sumOfWeights += weights[j];
            }
            for (int j = 0; j < numberOfWaypoints; j++)
            {
               waypointTimes[j] = weights[j] / sumOfWeights;
               if (j > 0)
                  waypointTimes[j] += waypointTimes[j - 1];
            }
         }

         solver.clearWaypoints();
         solver.addWaypoint(0, startPosition, 0);

         for (int j = 0; j < numberOfWaypoints; j++)
         {
            solver.addWaypointPosition(waypointTimes[j], waypointPositions[j]);
         }
         solver.addWaypoint(1, endPosition, 0);

         solver.solve();
         DMatrixRMaj solution = solver.getSolution();

         int waypointIndex = random.nextInt(numberOfWaypoints);
         double waypointTime = waypointTimes[waypointIndex];
         double expectedVelocity = computeExpectedVelocity(waypointTime, waypointIndex, solution);
         double actualVelocity = solver.computeVelocity(solver.getWaypoint(waypointIndex + 1).getTime());
         assertEquals(expectedVelocity, actualVelocity, 1.0e-10 * Math.max(1.0, Math.abs(expectedVelocity)));
      }
   }

   @Test
   public void testGetPosition()
   {
      Random random = new Random(45435);
      MultiSpline1DSolver solver = new MultiSpline1DSolver();

      for (int i = 0; i < 100; i++)
      {
         int numberOfWaypoints = random.nextInt(10) + 1;

         double startPosition = EuclidCoreRandomTools.nextDouble(random);
         double endPosition = EuclidCoreRandomTools.nextDouble(random);
         double startVelocity = EuclidCoreRandomTools.nextDouble(random);
         double endVelocity = EuclidCoreRandomTools.nextDouble(random);
         double[] waypointPositions = random.doubles(numberOfWaypoints, -0.5, 10.0).toArray();
         double[] waypointTimes = new double[numberOfWaypoints];
         { // Computing the waypoint times
            double[] weights = new double[numberOfWaypoints + 1];
            double sumOfWeights = 0.0;
            for (int j = 0; j < weights.length; j++)
            {
               weights[j] = random.nextDouble();
               sumOfWeights += weights[j];
            }
            for (int j = 0; j < numberOfWaypoints; j++)
            {
               waypointTimes[j] = weights[j] / sumOfWeights;
               if (j > 0)
                  waypointTimes[j] += waypointTimes[j - 1];
            }
         }

         solver.clearWaypoints();
         solver.addWaypoint(0, startPosition, startVelocity);

         for (int j = 0; j < numberOfWaypoints; j++)
         {
            solver.addWaypointPosition(waypointTimes[j], waypointPositions[j]);
         }
         solver.addWaypoint(1, endPosition, endVelocity);

         solver.solve();

         assertEquals(startPosition, solver.computePosition(0.0), 1.0e-9 * Math.max(1.0, Math.abs(startPosition)));
         assertEquals(startPosition, solver.computePosition(-0.1), 1.0e-9 * Math.max(1.0, Math.abs(startPosition)));
         assertEquals(endPosition, solver.computePosition(1.0), 1.0e-8 * Math.max(1.0, Math.abs(endPosition)));
         assertEquals(endPosition, solver.computePosition(1.1), 1.0e-8 * Math.max(1.0, Math.abs(endPosition)));

         for (int segmentIndex = 0; segmentIndex < numberOfWaypoints; segmentIndex++)
         {
            int firstWaypointIndex = segmentIndex;
            double firstWaypointTime = waypointTimes[firstWaypointIndex];
            double secondWaypointTime = segmentIndex >= (numberOfWaypoints - 1) ? 1.0 : waypointTimes[firstWaypointIndex + 1];

            double expectedPosition = computeExpectedPosition(firstWaypointTime, firstWaypointIndex, solver.getSolution());
            double actualPosition = solver.computePosition(firstWaypointTime);
            assertEquals(expectedPosition, actualPosition, 1.0e-9 * Math.max(1.0, Math.abs(expectedPosition)));
            assertEquals(waypointPositions[firstWaypointIndex], expectedPosition, 1.0e-7 * Math.max(1.0, Math.abs(waypointPositions[firstWaypointIndex])));

            double time = EuclidCoreRandomTools.nextDouble(random, firstWaypointTime, secondWaypointTime);
            expectedPosition = computeExpectedPosition(time, firstWaypointIndex + 1, solver.getSolution());
            actualPosition = solver.computePosition(time);
            assertEquals(expectedPosition, actualPosition, 1.0e-9 * Math.max(1.0, Math.abs(expectedPosition)));
         }
      }
   }

   @Test
   public void testGetVelocity()
   {
      Random random = new Random(45435);
      MultiSpline1DSolver solver = new MultiSpline1DSolver();

      for (int i = 0; i < 100; i++)
      {
         int numberOfWaypoints = random.nextInt(10) + 1;

         double startPosition = EuclidCoreRandomTools.nextDouble(random);
         double endPosition = EuclidCoreRandomTools.nextDouble(random);
         double startVelocity = EuclidCoreRandomTools.nextDouble(random);
         double endVelocity = EuclidCoreRandomTools.nextDouble(random);
         double[] waypointPositions = random.doubles(numberOfWaypoints, -0.5, 10.0).toArray();
         double[] waypointTimes = new double[numberOfWaypoints];
         { // Computing the waypoint times
            double[] weights = new double[numberOfWaypoints + 1];
            double sumOfWeights = 0.0;
            for (int j = 0; j < weights.length; j++)
            {
               weights[j] = random.nextDouble();
               sumOfWeights += weights[j];
            }
            for (int j = 0; j < numberOfWaypoints; j++)
            {
               waypointTimes[j] = weights[j] / sumOfWeights;
               if (j > 0)
                  waypointTimes[j] += waypointTimes[j - 1];
            }
         }

         solver.clearWaypoints();
         solver.addWaypoint(0, startPosition, startVelocity);

         for (int j = 0; j < numberOfWaypoints; j++)
         {
            solver.addWaypointPosition(waypointTimes[j], waypointPositions[j]);
         }
         solver.addWaypoint(1, endPosition, endVelocity);

         solver.solve();

         assertEquals(startVelocity, solver.computeVelocity(0.0), 1.0e-8 * Math.max(1.0, Math.abs(startVelocity)));
         assertEquals(startVelocity, solver.computeVelocity(-0.1), 1.0e-8 * Math.max(1.0, Math.abs(startVelocity)));
         assertEquals(endVelocity, solver.computeVelocity(1.0), 1.0e-8 * Math.max(1.0, Math.abs(endVelocity)));
         assertEquals(endVelocity, solver.computeVelocity(1.1), 1.0e-8 * Math.max(1.0, Math.abs(endVelocity)));

         for (int segmentIndex = 0; segmentIndex < numberOfWaypoints; segmentIndex++)
         {
            int firstWaypointIndex = segmentIndex;
            double firstWaypointTime = waypointTimes[firstWaypointIndex];
            double secondWaypointTime = segmentIndex >= (numberOfWaypoints - 1) ? 1.0 : waypointTimes[firstWaypointIndex + 1];

            double expectedVelocity = computeExpectedVelocity(firstWaypointTime, firstWaypointIndex, solver.getSolution());
            double actualVelocity = solver.computeVelocity(firstWaypointTime);
            assertEquals(expectedVelocity, actualVelocity, 1.0e-10 * Math.max(1.0, Math.abs(expectedVelocity)));

            double time = EuclidCoreRandomTools.nextDouble(random, firstWaypointTime, secondWaypointTime);
            expectedVelocity = computeExpectedVelocity(time, firstWaypointIndex + 1, solver.getSolution());
            actualVelocity = solver.computeVelocity(time);
            assertEquals(expectedVelocity, actualVelocity, 1.0e-10 * Math.max(1.0, Math.abs(expectedVelocity)));
         }
      }
   }

   private static double computeExpectedPosition(double time, int waypointIndex, DMatrixRMaj solution)
   {
      DMatrixRMaj tempLine = new DMatrixRMaj(1, defaultCoefficients);
      DMatrixRMaj tempCoeffs = new DMatrixRMaj(defaultCoefficients, 1);
      MultiSpline1DSolver.getPositionConstraintABlock(time, defaultCoefficients, 0, 0, tempLine);
      int index = waypointIndex * defaultCoefficients;
      CommonOps_DDRM.extract(solution, index, index + defaultCoefficients, 0, 1, tempCoeffs, 0, 0);
      return CommonOps_DDRM.dot(tempCoeffs, tempLine);
   }

   private static double computeExpectedVelocity(double waypointTime, int waypointIndex, DMatrixRMaj solution)
   {
      DMatrixRMaj tempLine = new DMatrixRMaj(1, defaultCoefficients);
      DMatrixRMaj tempCoeffs = new DMatrixRMaj(defaultCoefficients, 1);
      MultiSpline1DSolver.getVelocityConstraintABlock(waypointTime, defaultCoefficients, 0, 0, tempLine);
      int index = waypointIndex * defaultCoefficients;
      CommonOps_DDRM.extract(solution, index, index + defaultCoefficients, 0, 1, tempCoeffs, 0, 0);
      return CommonOps_DDRM.dot(tempCoeffs, tempLine);
   }

   public static Function wrap(BaseFunctionGenerator functionGenerator)
   {
      return new Function()
      {
         private double lastTime = 0.0;

         @Override
         public void compute(double t)
         {
            functionGenerator.integrateAngle(t - lastTime);
         }

         @Override
         public double getValue()
         {
            return functionGenerator.getValue();
         }

         @Override
         public double getValueDot()
         {
            return functionGenerator.getValueDot();
         }
      };
   }

   public static MultiSpline1DSolver nextFunctionBasedMultiSpline(Random random, Function function)
   {
      MultiSpline1DSolver next = new MultiSpline1DSolver();
      int numberOfWaypoints = 2 + random.nextInt(100);

      double t0 = random.nextDouble();
      double duration = EuclidCoreRandomTools.nextDouble(random, 0.3, 2.0);
      double tf = t0 + duration;
      double[] times = new double[numberOfWaypoints];
      times[0] = t0;
      times[numberOfWaypoints - 1] = tf;

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         double meanInterval = duration / (numberOfWaypoints - 1);
         double refTime = t0 + i * meanInterval;
         times[i] = refTime + EuclidCoreRandomTools.nextDouble(random, 0.5 * meanInterval);
      }

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         function.compute(times[i]);
         double x = function.getValue();
         double xd = function.getValueDot();
         next.addWaypoint().set(times[i], x, xd);
      }

      return next;
   }

   static void plot(MultiSpline1DSolver solver, Function function)
   {
      plot(solver, function, true);
   }

   static void plot(MultiSpline1DSolver solver, Function function, boolean wait)
   {
      int numTicks = 5000;
      double[] time = new double[numTicks];
      double[] x_solver = new double[numTicks];
      double t0 = solver.getFirstWaypoint().getTime();
      double tf = solver.getLastWaypoint().getTime();

      for (int i = 0; i < numTicks; i++)
      {
         double t = EuclidCoreTools.interpolate(t0, tf, i / (numTicks - 1.0));
         time[i] = t;
         x_solver[i] = solver.computePosition(t);
      }

      double[] x_function = null;
      if (function != null)
      {
         x_function = new double[numTicks];
         for (int i = 0; i < numTicks; i++)
         {
            double t = EuclidCoreTools.interpolate(t0, tf, i / (numTicks - 1.0));
            function.compute(t);
            x_function[i] = function.getValue();
         }
      }

      MatlabChart fig = new MatlabChart();
      if (function != null)
         fig.plot(time, x_function, ":b", 1.0f, "function");
      fig.plot(time, x_solver, "-r", 2.0f, "solver");
      fig.RenderPlot(); // First render plot before modifying
      fig.title("Solver output"); // title('Stock 1 vs. Stock 2');
      fig.xlabel("t"); // xlabel('Days');
      fig.ylabel("x"); // ylabel('Price');
      fig.grid("on", "on"); // grid on;
      fig.legend("northeast"); // legend('AAPL','BAC','Location','northeast')
      fig.font("Helvetica", 15); // .. 'FontName','Helvetica','FontSize',15
      fig.displayInJFrame(wait);
   }

   static interface Function
   {
      void compute(double t);

      double getValue();

      double getValueDot();
   }

   static class SineFunction implements Function
   {
      private double amplitude, frequency, offset, phase;
      private double t = 0.0;

      public SineFunction(double amplitude, double frequency, double offset, double phase)
      {
         this.amplitude = amplitude;
         this.frequency = frequency;
         this.offset = offset;
         this.phase = phase;
      }

      public SineFunction(Random random)
      {
         amplitude = random.nextDouble();
         frequency = 1.0 + 2.0 * random.nextDouble();
         offset = random.nextDouble();
         phase = EuclidCoreRandomTools.nextDouble(random, Math.PI);
      }

      @Override
      public void compute(double t)
      {
         this.t = t;
      }

      @Override
      public double getValue()
      {
         return offset + amplitude * Math.sin(2.0 * Math.PI * frequency * t + phase);
      }

      @Override
      public double getValueDot()
      {
         return 2.0 * Math.PI * frequency * amplitude * Math.cos(2.0 * Math.PI * frequency * t + phase);
      }
   }
}
