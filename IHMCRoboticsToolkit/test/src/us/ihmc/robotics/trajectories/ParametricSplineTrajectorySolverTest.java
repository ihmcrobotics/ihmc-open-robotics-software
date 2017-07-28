package us.ihmc.robotics.trajectories;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * Created by agrabertilton on 2/10/15.
 */
public class ParametricSplineTrajectorySolverTest
{
   /**
    * Tested for third order splines, but not for others
    * @Agraber
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test3rdOrder2DTrajectoryWithMidpoint(){
      double tolerance = 1e-13;

      int order = 3;
      int continuityConstraints = 3;
      int numberOfVariables = 2;
      int numberOfConstraints = 5;

      double startTime = 0;
      double[] startPosition = {0.0, 0.0};
      double[] startVelocity = {0.0, 0.0};

      double endTime = 0.5;
      double[] endPosition = {0.1, 0.0};
      double[] endVelocity = {0.0, 0.0};

      double midPointTime = (startTime + endTime)/2.0;
      double[] midPointPosition = {1.0, 0.5};

      ParametricSplineTrajectorySolver solver = new ParametricSplineTrajectorySolver(order, continuityConstraints, numberOfConstraints, numberOfVariables);
      double[] times = new double[solver.getNumberOfSplines() + 1];
      for (int i = 0; i < times.length; i++){
         times[i] = Double.NaN;
      }
      times[0] = startTime;
      times[times.length-1] = endTime;
      solver.setTimes(times);

      solver.setPositionConstraint(startTime, startPosition);
      solver.setVelocityConstraint(startTime, startVelocity);
      solver.setPositionConstraint(endTime, endPosition);
      solver.setVelocityConstraint(endTime, endVelocity);
      solver.setPositionConstraint(midPointTime, midPointPosition);

      ParametricSplineTrajectory trajectory = solver.computeTrajectory();
      assertArrayEquals(startPosition, trajectory.getPositions(startTime), tolerance);
      assertArrayEquals(startVelocity, trajectory.getVelocities(startTime), tolerance);
      assertArrayEquals(endPosition, trajectory.getPositions(endTime), tolerance);
      assertArrayEquals(endVelocity, trajectory.getVelocities(endTime), tolerance);
      assertArrayEquals(midPointPosition, trajectory.getPositions(midPointTime), tolerance);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test3rdOrder1DTrajectoryWithMidpoint(){
      double tolerance = 1e-13;

      int order = 3;
      int continuityConstraints = 3;
      int numberOfVariables = 1;
      int numberOfConstraints = 5;

      double startTime = 0;
      double[] startPosition = {0.0};
      double[] startVelocity = {0.0};

      double endTime = 0.5;
      double[] endPosition = {0.1};
      double[] endVelocity = {0.0};

      double midPointTime = (startTime + endTime)/2.0;
      double[] midPointPosition = {1.0};

      ParametricSplineTrajectorySolver solver = new ParametricSplineTrajectorySolver(order, continuityConstraints, numberOfConstraints, numberOfVariables);
      double[] times = new double[solver.getNumberOfSplines() + 1];
      for (int i = 0; i < times.length; i++){
         times[i] = Double.NaN;
      }
      times[0] = startTime;
      times[times.length-1] = endTime;

      solver.setTimes(times);

      solver.setPositionConstraint(startTime, startPosition);
      solver.setVelocityConstraint(startTime, startVelocity);
      solver.setPositionConstraint(endTime, endPosition);
      solver.setVelocityConstraint(endTime, endVelocity);
      solver.setPositionConstraint(midPointTime, midPointPosition);

      ParametricSplineTrajectory trajectory = solver.computeTrajectory();
      assertArrayEquals(startPosition, trajectory.getPositions(startTime), tolerance);
      assertArrayEquals(startVelocity, trajectory.getVelocities(startTime), tolerance);
      assertArrayEquals(endPosition, trajectory.getPositions(endTime), tolerance);
      assertArrayEquals(endVelocity, trajectory.getVelocities(endTime), tolerance);
      assertArrayEquals(midPointPosition, trajectory.getPositions(midPointTime), tolerance);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test3rdOrderBasic1DMovementTrajectory(){
      double tolerance = 1e-13;

      int order = 3;
      int continuityConstraints = 3;
      int numberOfVariables = 1;
      int numberOfConstraints = 4;

      double startTime = 0;
      double[] startPosition = {0.0};
      double[] startVelocity = {0.0};

      double endTime = 0.5;
      double[] endPosition = {1.0};
      double[] endVelocity = {0.0};

      ParametricSplineTrajectorySolver solver = new ParametricSplineTrajectorySolver(order, continuityConstraints, numberOfConstraints, numberOfVariables);
      double[] times = new double[solver.getNumberOfSplines() + 1];
      for (int i = 0; i < times.length; i++){
         times[i] = Double.NaN;
      }
      times[0] = startTime;
      times[times.length-1] = endTime;

      solver.setTimes(times);

      solver.setPositionConstraint(startTime, startPosition);
      solver.setVelocityConstraint(startTime, startVelocity);
      solver.setPositionConstraint(endTime, endPosition);
      solver.setVelocityConstraint(endTime, endVelocity);

      ParametricSplineTrajectory trajectory = solver.computeTrajectory();
      assertArrayEquals(startPosition, trajectory.getPositions(startTime), tolerance);
      assertArrayEquals(startVelocity, trajectory.getVelocities(startTime), tolerance);
      assertArrayEquals(endPosition, trajectory.getPositions(endTime), tolerance);
      assertArrayEquals(endVelocity, trajectory.getVelocities(endTime), tolerance);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test3rdOrderNoMovementTrajectory(){
      double tolerance = 1e-13;

      int order = 3;
      int continuityConstraints = 3;
      int numberOfVariables = 1;
      int numberOfConstraints = 4;

      double startTime = 0;
      double[] startPosition = new double[1];
      double[] startVelocity = new double[1];

      double endTime = 0.5;
      double[] endPosition = new double[1];
      double[] endVelocity = new double[1];

      ParametricSplineTrajectorySolver solver = new ParametricSplineTrajectorySolver(order, continuityConstraints, numberOfConstraints, numberOfVariables);
      double[] times = new double[solver.getNumberOfSplines() + 1];
      for (int i = 0; i < times.length; i++){
         times[i] = Double.NaN;
      }
      times[0] = startTime;
      times[times.length-1] = endTime;

      solver.setTimes(times);

      solver.setPositionConstraint(startTime, startPosition);
      solver.setVelocityConstraint(startTime, startVelocity);
      solver.setPositionConstraint(endTime, endPosition);
      solver.setVelocityConstraint(endTime, endVelocity);

      ParametricSplineTrajectory trajectory = solver.computeTrajectory();
      assertArrayEquals(startPosition, trajectory.getPositions(startTime), tolerance);
      assertArrayEquals(startVelocity, trajectory.getVelocities(startTime), tolerance);
      assertArrayEquals(endPosition, trajectory.getPositions(endTime), tolerance);
      assertArrayEquals(endVelocity, trajectory.getVelocities(endTime), tolerance);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInvalidNumberOfConstraints(){
      int order = 3;
      int continuityConstraints = 3;
      int numberOfVariables = 1;
      int numberOfConstraints = 2;

      double startTime = 0;
      double[] startPosition = new double[1];
      double endTime = 0;
      double[] endPosition = new double[1];

      boolean errorThrown = false;
      try{
         ParametricSplineTrajectorySolver solver = new ParametricSplineTrajectorySolver(order, continuityConstraints, numberOfConstraints, numberOfVariables);
      }catch (Exception e){
         errorThrown = true;
      }
      assertTrue(errorThrown);
   }
}
