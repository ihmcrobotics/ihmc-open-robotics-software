package us.ihmc.robotics.math;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.robotics.math.trajectories.YoTrajectory;
import us.ihmc.robotics.math.trajectories.YoTrajectory3D;
import us.ihmc.robotics.math.trajectories.YoSegmentedFrameTrajectory3D;
import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class TrajectoryMathToolsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static YoVariableRegistry registry = new YoVariableRegistry("TrajectoryMathTestRegistry");

   private static final double epsilon = 1e-6;

   @Before
   public void setupTest()
   {
      registry.clear();
   }

   @After
   public void finishTest()
   {

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrajectoryAddition()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(1, 10, 1.5, -2.5);
      traj2.setCubic(1, 10, 12.5, 3.5);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      TrajectoryMathTools.add(traj1, traj1, traj2);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj1.getCoefficient(0), 13.482853223593963, epsilon);
      Assert.assertEquals(traj1.getCoefficient(1), 1.069958847736625, epsilon);
      Assert.assertEquals(traj1.getCoefficient(2), -0.588477366255144, epsilon);
      Assert.assertEquals(traj1.getCoefficient(3), 0.03566529492455418, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrajectoryMultiTimeScaleAddition()
   {
      List<YoTrajectory> resultTrajectoryList = new ArrayList<>(3);
      resultTrajectoryList.add(new YoTrajectory("ResultTraj0", 6, registry));
      resultTrajectoryList.add(new YoTrajectory("ResultTraj1", 6, registry));
      resultTrajectoryList.add(new YoTrajectory("ResultTraj2", 6, registry));
      resultTrajectoryList.add(new YoTrajectory("ResultTraj3", 6, registry));
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(1, 6, 1.5, -2.5);
      traj2.setCubic(3, 5, 12.5, 3.5);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      int numberOfSegments = TrajectoryMathTools.add(resultTrajectoryList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      YoTrajectory traj3 = resultTrajectoryList.get(0);
      YoTrajectory traj4 = resultTrajectoryList.get(1);
      YoTrajectory traj5 = resultTrajectoryList.get(2);
      Assert.assertEquals(traj3.getInitialTime(), 1.0, epsilon);
      Assert.assertEquals(traj3.getFinalTime(), 3.0, epsilon);
      Assert.assertTrue(traj3.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj3.getCoefficient(0), 0.9560, epsilon);
      Assert.assertEquals(traj3.getCoefficient(1), 1.1520, epsilon);
      Assert.assertEquals(traj3.getCoefficient(2), -0.6720, epsilon);
      Assert.assertEquals(traj3.getCoefficient(3), 0.0640, epsilon);

      Assert.assertEquals(traj4.getInitialTime(), 3.0, epsilon);
      Assert.assertEquals(traj4.getFinalTime(), 5.0, epsilon);
      Assert.assertTrue(traj4.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj4.getCoefficient(0), 0.9560 - 109.0000, epsilon);
      Assert.assertEquals(traj4.getCoefficient(1), 1.1520 + 101.2500, epsilon);
      Assert.assertEquals(traj4.getCoefficient(2), -0.6720 - 27.0000, epsilon);
      Assert.assertEquals(traj4.getCoefficient(3), 0.0640 + 2.2500, epsilon);

      Assert.assertEquals(traj5.getInitialTime(), 5.0, epsilon);
      Assert.assertEquals(traj5.getFinalTime(), 6.0, epsilon);
      Assert.assertTrue(traj5.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj5.getCoefficient(0), 0.9560, epsilon);
      Assert.assertEquals(traj5.getCoefficient(1), 1.1520, epsilon);
      Assert.assertEquals(traj5.getCoefficient(2), -0.6720, epsilon);
      Assert.assertEquals(traj5.getCoefficient(3), 0.0640, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMultiTimeScaleOperation()
   {
      List<Double> timeList = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0));
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 10, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 10, registry);
      traj1.setConstant(5, 10, 1);
      traj2.setConstant(1, 4, 1);
      int numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 1);
      Assert.assertTrue(timeList.get(1) == 4);
      Assert.assertTrue(timeList.get(2) == 5);
      Assert.assertTrue(timeList.get(3) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(1, 6, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 1);
      Assert.assertTrue(timeList.get(1) == 5);
      Assert.assertTrue(timeList.get(2) == 6);
      Assert.assertTrue(timeList.get(3) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(1, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 1);
      Assert.assertTrue(timeList.get(1) == 5);
      Assert.assertTrue(timeList.get(2) == 10);
      Assert.assertTrue(timeList.get(3) == 11);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(6, 9, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 6);
      Assert.assertTrue(timeList.get(2) == 9);
      Assert.assertTrue(timeList.get(3) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(6, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 6);
      Assert.assertTrue(timeList.get(2) == 10);
      Assert.assertTrue(timeList.get(3) == 11);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(11, 12, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 10);
      Assert.assertTrue(timeList.get(2) == 11);
      Assert.assertTrue(timeList.get(3) == 12);

      // 
      traj2.setConstant(5, 10, 1);
      traj1.setConstant(1, 4, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 1);
      Assert.assertTrue(timeList.get(1) == 4);
      Assert.assertTrue(timeList.get(2) == 5);
      Assert.assertTrue(timeList.get(3) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(1, 6, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 1);
      Assert.assertTrue(timeList.get(1) == 5);
      Assert.assertTrue(timeList.get(2) == 6);
      Assert.assertTrue(timeList.get(3) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(1, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 1);
      Assert.assertTrue(timeList.get(1) == 5);
      Assert.assertTrue(timeList.get(2) == 10);
      Assert.assertTrue(timeList.get(3) == 11);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(6, 9, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 6);
      Assert.assertTrue(timeList.get(2) == 9);
      Assert.assertTrue(timeList.get(3) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(6, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 6);
      Assert.assertTrue(timeList.get(2) == 10);
      Assert.assertTrue(timeList.get(3) == 11);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(11, 12, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 10);
      Assert.assertTrue(timeList.get(2) == 11);
      Assert.assertTrue(timeList.get(3) == 12);

      //
      traj1.setConstant(5, 10, 1);
      traj2.setConstant(5, 9, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 9);
      Assert.assertTrue(timeList.get(2) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(5, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 10);
      Assert.assertTrue(timeList.get(2) == 11);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(4, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Assert.assertTrue(timeList.get(0) == 4);
      Assert.assertTrue(timeList.get(1) == 5);
      Assert.assertTrue(timeList.get(2) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(6, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 6);
      Assert.assertTrue(timeList.get(2) == 10);

      //
      traj2.setConstant(5, 10, 1);
      traj1.setConstant(5, 9, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 9);
      Assert.assertTrue(timeList.get(2) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(5, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 10);
      Assert.assertTrue(timeList.get(2) == 11);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(4, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Assert.assertTrue(timeList.get(0) == 4);
      Assert.assertTrue(timeList.get(1) == 5);
      Assert.assertTrue(timeList.get(2) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(6, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 6);
      Assert.assertTrue(timeList.get(2) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(5, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 1);
      Assert.assertTrue(timeList.get(0) == 5);
      Assert.assertTrue(timeList.get(1) == 10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrajectorySubtraction()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(0, 2, 3.5, -2.5);
      traj2.setCubic(0, 2, -0.5, -0.6);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      TrajectoryMathTools.subtract(traj1, traj1, traj2);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj1.getCoefficient(0), 3.5 + 0.500, epsilon);
      Assert.assertEquals(traj1.getCoefficient(1), 0.0 - 0.000, epsilon);
      Assert.assertEquals(traj1.getCoefficient(2), -4.5 + 0.075, epsilon);
      Assert.assertEquals(traj1.getCoefficient(3), 1.5 - 0.025, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrajectoryMultiTimeScaleSubtraction()
   {
      List<YoTrajectory> resultTrajectoryList = new ArrayList<>(3);
      resultTrajectoryList.add(new YoTrajectory("ResultTraj0", 6, registry));
      resultTrajectoryList.add(new YoTrajectory("ResultTraj1", 6, registry));
      resultTrajectoryList.add(new YoTrajectory("ResultTraj2", 6, registry));
      resultTrajectoryList.add(new YoTrajectory("ResultTraj3", 6, registry));
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(0, 6, 2.1, 10.5);
      traj2.setCubic(1, 10, -10.6, 6.5);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      int numberOfSegments = TrajectoryMathTools.subtract(resultTrajectoryList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      YoTrajectory traj3 = resultTrajectoryList.get(0);
      YoTrajectory traj4 = resultTrajectoryList.get(1);
      YoTrajectory traj5 = resultTrajectoryList.get(2);
      Assert.assertEquals(traj3.getInitialTime(), 0.0, epsilon);
      Assert.assertEquals(traj3.getFinalTime(), 1.0, epsilon);
      Assert.assertTrue(traj3.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj3.getCoefficient(0), 2.1000, epsilon);
      Assert.assertEquals(traj3.getCoefficient(1), 0.0000, epsilon);
      Assert.assertEquals(traj3.getCoefficient(2), 0.7000, epsilon);
      Assert.assertEquals(traj3.getCoefficient(3), -0.0777778, epsilon);

      Assert.assertEquals(traj4.getInitialTime(), 1.0, epsilon);
      Assert.assertEquals(traj4.getFinalTime(), 6.0, epsilon);
      Assert.assertTrue(traj4.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj4.getCoefficient(0), 2.1000 + 9.91975309, epsilon);
      Assert.assertEquals(traj4.getCoefficient(1), 0.0000 + 1.407407407, epsilon);
      Assert.assertEquals(traj4.getCoefficient(2), 0.7000 - 0.77407407, epsilon);
      Assert.assertEquals(traj4.getCoefficient(3), -0.077777778 + 0.0469135802, epsilon);

      Assert.assertEquals(traj5.getInitialTime(), 6.0, epsilon);
      Assert.assertEquals(traj5.getFinalTime(), 10.0, epsilon);
      Assert.assertTrue(traj5.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj5.getCoefficient(0), 9.91975309, epsilon);
      Assert.assertEquals(traj5.getCoefficient(1), 1.407407407, epsilon);
      Assert.assertEquals(traj5.getCoefficient(2), -0.77407407, epsilon);
      Assert.assertEquals(traj5.getCoefficient(3), 0.0469135802, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrajectoryMultiplication2()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 8, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 8, registry);
      traj1.setLinear(0, 10, 1, 2);
      traj2.setLinear(0, 10, 2, 3);
      TrajectoryMathTools.multiply(traj1, traj1, traj2);
      Assert.assertEquals(traj1.getCoefficient(0), 2, epsilon);
      Assert.assertEquals(traj1.getCoefficient(1), 0.3, epsilon);
      Assert.assertEquals(traj1.getCoefficient(2), 0.01, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrajectoryMultiplication()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(12, 15, 19.5, 200.5);
      traj2.setCubic(12, 15, 0.5, 0.1);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      TrajectoryMathTools.multiply(traj1, traj1, traj2);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 7);
      Assert.assertEquals(traj1.getCoefficient(0), -2.228097449999825e+06, epsilon);
      Assert.assertEquals(traj1.getCoefficient(1), 1.016083999999920e+06, epsilon);
      Assert.assertEquals(traj1.getCoefficient(2), -1.920462999999848e+05, epsilon);
      Assert.assertEquals(traj1.getCoefficient(3), 1.925763703703550e+04, epsilon);
      Assert.assertEquals(traj1.getCoefficient(4), -1.080637037036950e+03, epsilon);
      Assert.assertEquals(traj1.getCoefficient(5), 32.177777777775184, epsilon);
      Assert.assertEquals(traj1.getCoefficient(6), -0.397256515775002, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrajectoryMultiTimeScaleMultiplication()
   {
      List<YoTrajectory> resultTrajectoryList = new ArrayList<>(2);
      resultTrajectoryList.add(new YoTrajectory("ResultTraj0", 7, registry));
      resultTrajectoryList.add(new YoTrajectory("ResultTraj1", 7, registry));
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(1, 6, 1.1, 2.5);
      traj2.setCubic(1, 10, -1.6, 0.5);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      int numberOfSegments = TrajectoryMathTools.multiply(resultTrajectoryList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      YoTrajectory traj3 = resultTrajectoryList.get(0);
      YoTrajectory traj4 = resultTrajectoryList.get(1);
      Assert.assertEquals(traj3.getInitialTime(), 1.0, epsilon);
      Assert.assertEquals(traj3.getFinalTime(), 6.0, epsilon);
      Assert.assertTrue(traj3.getNumberOfCoefficients() == 7);
      Assert.assertEquals(traj3.getCoefficient(0), -1.956841152263374, epsilon);
      Assert.assertEquals(traj3.getCoefficient(1), 0.388404938271605, epsilon);
      Assert.assertEquals(traj3.getCoefficient(2), -0.164315061728395, epsilon);
      Assert.assertEquals(traj3.getCoefficient(3), -0.052446419753086, epsilon);
      Assert.assertEquals(traj3.getCoefficient(4), 0.028553086419753, epsilon);
      Assert.assertEquals(traj3.getCoefficient(5), -0.003484444444444, epsilon);
      Assert.assertEquals(traj3.getCoefficient(6), 0.000129053497942, epsilon);

      Assert.assertEquals(traj4.getInitialTime(), 6.0, epsilon);
      Assert.assertEquals(traj4.getFinalTime(), 10.0, epsilon);
      Assert.assertTrue(traj4.getNumberOfCoefficients() == 4);
      Assert.assertEquals(traj4.getCoefficient(0), 0, epsilon);
      Assert.assertEquals(traj4.getCoefficient(1), 0, epsilon);
      Assert.assertEquals(traj4.getCoefficient(2), 0, epsilon);
      Assert.assertEquals(traj4.getCoefficient(3), 0, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test3DTrajectoryAddition()
   {
      YoTrajectory3D traj1 = new YoTrajectory3D("Trajectory1", 3, registry);
      YoTrajectory3D traj2 = new YoTrajectory3D("Trajectory2", 3, registry);
      traj1.setLinear(0, 1, new Point3D(0.5, 0.1, 10), new Point3D(1, 10, 5));
      traj2.setLinear(0, 1, new Point3D(), new Point3D(5, 7.7, 1));
      TrajectoryMathTools.add(traj1, traj1, traj2);

      YoTrajectory traj = traj1.getYoTrajectoryX();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      Assert.assertEquals(traj.getCoefficient(0), 0.5, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), 5.5, epsilon);

      traj = traj1.getYoTrajectoryY();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      Assert.assertEquals(traj.getCoefficient(0), 0.1, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), 9.9 + 7.7, epsilon);

      traj = traj1.getYoTrajectoryZ();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      Assert.assertEquals(traj.getCoefficient(0), 10, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), -4, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test3DTrajectorySubtraction()
   {
      YoTrajectory3D traj1 = new YoTrajectory3D("Trajectory1", 3, registry);
      YoTrajectory3D traj2 = new YoTrajectory3D("Trajectory2", 3, registry);
      traj1.setLinear(0, 1, new Point3D(0.1, 3.414, 1.87), new Point3D(2.09, 1.35, 5.35));
      traj2.setLinear(0, 1, new Point3D(3.14, 1.59, 12.9), new Point3D(4.51, 5.32, 1.12));
      TrajectoryMathTools.subtract(traj1, traj1, traj2);

      YoTrajectory traj = traj1.getYoTrajectoryX();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      Assert.assertEquals(traj.getCoefficient(0), -3.04, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), 0.62, epsilon);

      traj = traj1.getYoTrajectoryY();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      Assert.assertEquals(traj.getCoefficient(0), 1.824, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), -5.794, epsilon);

      traj = traj1.getYoTrajectoryZ();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      Assert.assertEquals(traj.getCoefficient(0), -11.03, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), 15.26, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test3DTrajectoryDotProducts()
   {
      YoTrajectory3D traj1 = new YoTrajectory3D("Trajectory1", 3, registry);
      YoTrajectory3D traj2 = new YoTrajectory3D("Trajectory2", 3, registry);
      traj1.setLinear(0, 1, new Point3D(1, 3, 5), new Point3D(6, 4, 2));
      traj2.setLinear(0, 1, new Point3D(2, 4, 6), new Point3D(5, 3, 1));
      TrajectoryMathTools.dotProduct(traj1, traj1, traj2);

      YoTrajectory traj = traj1.getYoTrajectoryX();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      Assert.assertEquals(traj.getCoefficient(0), 2, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), 13, epsilon);
      Assert.assertEquals(traj.getCoefficient(2), 15, epsilon);

      traj = traj1.getYoTrajectoryY();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      Assert.assertEquals(traj.getCoefficient(0), 12, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), 1, epsilon);
      Assert.assertEquals(traj.getCoefficient(2), -1, epsilon);

      traj = traj1.getYoTrajectoryZ();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      Assert.assertEquals(traj.getCoefficient(0), 30, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), -43, epsilon);
      Assert.assertEquals(traj.getCoefficient(2), 15, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test3DTrajectoryCrossProduct()
   {
      YoTrajectory3D traj1 = new YoTrajectory3D("Trajectory1", 3, registry);
      YoTrajectory3D traj2 = new YoTrajectory3D("Trajectory2", 3, registry);
      traj1.setLinear(0, 1, new Point3D(1, 3, 5), new Point3D(6, 4, 2));
      traj2.setLinear(0, 1, new Point3D(2, 4, 6), new Point3D(5, 3, 1));
      TrajectoryMathTools.crossProduct(traj1, traj1, traj2);

      YoTrajectory traj = traj1.getYoTrajectoryX();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      Assert.assertEquals(traj.getCoefficient(0), -2, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), 8, epsilon);
      Assert.assertEquals(traj.getCoefficient(2), -8, epsilon);

      traj = traj1.getYoTrajectoryY();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      Assert.assertEquals(traj.getCoefficient(0), 4, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), -16, epsilon);
      Assert.assertEquals(traj.getCoefficient(2), 16, epsilon);

      traj = traj1.getYoTrajectoryZ();
      Assert.assertEquals(traj.getInitialTime(), 0, epsilon);
      Assert.assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      Assert.assertEquals(traj.getCoefficient(0), -2, epsilon);
      Assert.assertEquals(traj.getCoefficient(1), 8, epsilon);
      Assert.assertEquals(traj.getCoefficient(2), -8, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntegration()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 2, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 3, registry);
      traj1.setLinear(1, 11, 4, 5);
      TrajectoryMathTools.getIntergal(traj2, traj1);
      Assert.assertEquals(traj2.getCoefficient(0), -4.00, epsilon);
      Assert.assertEquals(traj2.getCoefficient(1), 3.90, epsilon);
      Assert.assertEquals(traj2.getCoefficient(2), 0.05, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDifferentiation()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 3, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 2, registry);
      traj1.setQuadratic(1, 11, 4, 0, 5);
      TrajectoryMathTools.getDerivative(traj2, traj1);
      Assert.assertEquals(traj2.getCoefficient(0), traj1.getCoefficient(1), epsilon);
      Assert.assertEquals(traj2.getCoefficient(1), 2 * traj1.getCoefficient(2), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testShifting()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 3, registry);
      traj1.setDirectlyFast(0, 1);
      traj1.setDirectlyFast(1, 2);
      traj1.setDirectlyFast(2, 3);
      traj1.setTime(2, 4);
      TrajectoryMathTools.addTimeOffset(traj1, 2);
      Assert.assertEquals(traj1.getCoefficient(0), 9, epsilon);
      Assert.assertEquals(traj1.getCoefficient(1), -10, epsilon);
      Assert.assertEquals(traj1.getCoefficient(2), 3, epsilon);
      TrajectoryMathTools.addTimeOffset(traj1, -4);
      Assert.assertEquals(traj1.getCoefficient(0), 17, epsilon);
      Assert.assertEquals(traj1.getCoefficient(1), 14, epsilon);
      Assert.assertEquals(traj1.getCoefficient(2), 3, epsilon);
   }

   private class DummySegmentedTrajectory extends YoSegmentedFrameTrajectory3D
   {

      public DummySegmentedTrajectory(String name, int maxNumberOfSegments, int maxNumberOfCoefficients, YoVariableRegistry registry)
      {
         super(name, maxNumberOfSegments, maxNumberOfCoefficients, registry);
      }

      public void setSegment(double t0, double tFinal, FramePoint z0, FramePoint zf)
      {
         segments.get(numberOfSegments.getIntegerValue()).setLinear(t0, tFinal, z0, zf);
         numberOfSegments.increment();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSegmentedAddition()
   {
      DummySegmentedTrajectory traj1 = new DummySegmentedTrajectory("TestTraj1", 4, 2, registry);
      DummySegmentedTrajectory traj2 = new DummySegmentedTrajectory("TestTraj2", 4, 2, registry);
      DummySegmentedTrajectory traj3 = new DummySegmentedTrajectory("TestTraj3", 10, 2, registry);
      traj1.setSegment(0, 1, new FramePoint(worldFrame, 10, 11, 12), new FramePoint(worldFrame, 13, 14, 15));
      traj1.setSegment(1, 2, new FramePoint(worldFrame, 15, 20, 25), new FramePoint(worldFrame, 20, 25, 30));
      traj1.setSegment(2, 3, new FramePoint(worldFrame, 25, 28, 31), new FramePoint(worldFrame, 35, 38, 41));
      traj2.setSegment(0.5, 0.6, new FramePoint(worldFrame, 1, 2, 3), new FramePoint(worldFrame, 3, 2, 1));
      traj2.setSegment(1.2, 2.2, new FramePoint(worldFrame, 3, 2, 1), new FramePoint(worldFrame, 4, 5, 6));
      TrajectoryMathTools.addSegmentedTrajectories(traj3, traj1, traj2, Epsilons.ONE_BILLIONTH);
   }
}
