package us.ihmc.robotics.math.trajectories.generators;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.robotics.math.trajectories.generators.MultiCubicSpline1DSolver.coefficients;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.Assert;

public class MultiCubicSpline1DSolverTest
{
   private final double permissableError = 1e-3;

   @Test
   public void testNotUsingNativeCommonOps()
   {
      DMatrixRMaj mat = new DMatrixRMaj(1, 1);
      MultiCubicSpline1DSolver solverUsingNative = new MultiCubicSpline1DSolver(true);
      solverUsingNative.setEndpoints(0, 1, 0, 1);
      solverUsingNative.addWaypoint(2, 0.25);
      solverUsingNative.addWaypoint(-2, 0.75);
      double costUsingNative = solverUsingNative.solve(mat);

      mat = new DMatrixRMaj(1, 1);
      MultiCubicSpline1DSolver solverNotUsingNative = new MultiCubicSpline1DSolver(false);
      solverNotUsingNative.setEndpoints(0, 1, 0, 1);
      solverNotUsingNative.addWaypoint(2, 0.25);
      solverNotUsingNative.addWaypoint(-2, 0.75);
      double costNotUsingNative = solverNotUsingNative.solve(mat);

      double percentDiff = 100 * Math.abs(costUsingNative - costNotUsingNative) / costNotUsingNative;
      Assert.assertTrue(percentDiff < permissableError);
   }

   @Test
   public void testGetWaypointVelocityFromSolution()
   {
      Random random = new Random(45435);
      DMatrixRMaj solution = new DMatrixRMaj(2, 1);
      MultiCubicSpline1DSolver solver = new MultiCubicSpline1DSolver();

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

         solver.clearWeights();
         solver.clearWaypoints();
         solver.setEndpoints(startPosition, 0.0, endPosition, 0.0);

         for (int j = 0; j < numberOfWaypoints; j++)
         {
            solver.addWaypoint(waypointPositions[j], waypointTimes[j]);
         }

         solver.solve(solution);

         int waypointIndex = random.nextInt(numberOfWaypoints);
         double waypointTime = waypointTimes[waypointIndex];
         double expectedVelocity = computeExpectedVelocity(waypointTime, waypointIndex, solution);
         double actualVelocity = solver.computeWaypointVelocityFromSolution(waypointIndex, solution);
         assertEquals(expectedVelocity, actualVelocity, 1.0e-10 * Math.max(1.0, Math.abs(expectedVelocity)));
      }
   }

   @Test
   public void testGetPosition()
   {
      Random random = new Random(45435);
      DMatrixRMaj solution = new DMatrixRMaj(2, 1);
      MultiCubicSpline1DSolver solver = new MultiCubicSpline1DSolver();

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

         solver.clearWeights();
         solver.clearWaypoints();
         solver.setEndpoints(startPosition, startVelocity, endPosition, endVelocity);

         for (int j = 0; j < numberOfWaypoints; j++)
         {
            solver.addWaypoint(waypointPositions[j], waypointTimes[j]);
         }

         solver.solve(solution);

         assertEquals(startPosition, solver.computePosition(0.0, solution));
         assertEquals(startPosition, solver.computePosition(-0.1, solution));
         assertEquals(endPosition, solver.computePosition(1.0, solution));
         assertEquals(endPosition, solver.computePosition(1.1, solution));

         for (int segmentIndex = 0; segmentIndex < numberOfWaypoints; segmentIndex++)
         {
            int firstWaypointIndex = segmentIndex;
            double firstWaypointTime = waypointTimes[firstWaypointIndex];
            double secondWaypointTime = segmentIndex >= (numberOfWaypoints - 1) ? 1.0 : waypointTimes[firstWaypointIndex + 1];

            double expectedPosition = computeExpectedPosition(firstWaypointTime, firstWaypointIndex, solution);
            double actualPosition = solver.computePosition(firstWaypointTime, solution);
            assertEquals(expectedPosition, actualPosition, 1.0e-10 * Math.max(1.0, Math.abs(expectedPosition)));
            assertEquals(waypointPositions[firstWaypointIndex], expectedPosition, 1.0e-7 * Math.max(1.0, Math.abs(waypointPositions[firstWaypointIndex])));

            double time = EuclidCoreRandomTools.nextDouble(random, firstWaypointTime, secondWaypointTime);
            expectedPosition = computeExpectedPosition(time, firstWaypointIndex + 1, solution);
            actualPosition = solver.computePosition(time, solution);
            assertEquals(expectedPosition, actualPosition, 1.0e-10 * Math.max(1.0, Math.abs(expectedPosition)));
         }
      }
   }

   @Test
   public void testGetVelocity()
   {
      Random random = new Random(45435);
      DMatrixRMaj solution = new DMatrixRMaj(2, 1);
      MultiCubicSpline1DSolver solver = new MultiCubicSpline1DSolver();

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

         solver.clearWeights();
         solver.clearWaypoints();
         solver.setEndpoints(startPosition, startVelocity, endPosition, endVelocity);

         for (int j = 0; j < numberOfWaypoints; j++)
         {
            solver.addWaypoint(waypointPositions[j], waypointTimes[j]);
         }

         solver.solve(solution);

         assertEquals(startVelocity, solver.computeVelocity(0.0, solution));
         assertEquals(startVelocity, solver.computeVelocity(-0.1, solution));
         assertEquals(endVelocity, solver.computeVelocity(1.0, solution));
         assertEquals(endVelocity, solver.computeVelocity(1.1, solution));

         for (int segmentIndex = 0; segmentIndex < numberOfWaypoints; segmentIndex++)
         {
            int firstWaypointIndex = segmentIndex;
            double firstWaypointTime = waypointTimes[firstWaypointIndex];
            double secondWaypointTime = segmentIndex >= (numberOfWaypoints - 1) ? 1.0 : waypointTimes[firstWaypointIndex + 1];

            double expectedVelocity = computeExpectedVelocity(firstWaypointTime, firstWaypointIndex, solution);
            double actualVelocity = solver.computeVelocity(firstWaypointTime, solution);
            assertEquals(expectedVelocity, actualVelocity, 1.0e-10 * Math.max(1.0, Math.abs(expectedVelocity)));

            double time = EuclidCoreRandomTools.nextDouble(random, firstWaypointTime, secondWaypointTime);
            expectedVelocity = computeExpectedVelocity(time, firstWaypointIndex + 1, solution);
            actualVelocity = solver.computeVelocity(time, solution);
            assertEquals(expectedVelocity, actualVelocity, 1.0e-10 * Math.max(1.0, Math.abs(expectedVelocity)));
         }
      }
   }

   private static double computeExpectedPosition(double time, int waypointIndex, DMatrixRMaj solution)
   {
      DMatrixRMaj tempLine = new DMatrixRMaj(1, coefficients);
      DMatrixRMaj tempCoeffs = new DMatrixRMaj(coefficients, 1);
      MultiCubicSpline1DSolver.getPositionConstraintABlock(time, 0, 0, tempLine);
      int index = waypointIndex * coefficients;
      CommonOps_DDRM.extract(solution, index, index + coefficients, 0, 1, tempCoeffs, 0, 0);
      return CommonOps_DDRM.dot(tempCoeffs, tempLine);
   }

   private static double computeExpectedVelocity(double waypointTime, int waypointIndex, DMatrixRMaj solution)
   {
      DMatrixRMaj tempLine = new DMatrixRMaj(1, coefficients);
      DMatrixRMaj tempCoeffs = new DMatrixRMaj(coefficients, 1);
      MultiCubicSpline1DSolver.getVelocityConstraintABlock(waypointTime, 0, 0, tempLine);
      int index = waypointIndex * coefficients;
      CommonOps_DDRM.extract(solution, index, index + coefficients, 0, 1, tempCoeffs, 0, 0);
      return CommonOps_DDRM.dot(tempCoeffs, tempLine);
   }
}
