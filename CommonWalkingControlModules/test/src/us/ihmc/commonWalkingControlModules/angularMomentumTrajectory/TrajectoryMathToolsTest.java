package us.ihmc.commonWalkingControlModules.angularMomentumTrajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.TrajectoryMathTools;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory3D;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TrajectoryMathToolsTest
{
   private static YoVariableRegistry registry = new YoVariableRegistry("TrajectoryMathTestRegistry");
   private static final double EPSILON = Epsilons.ONE_THOUSANDTH;
   
   @Before
   public void setupTest()
   {
      registry.clear();
   }

   @After
   public void finishTest()
   {

   }

   @Test
   public void testTrajectoryAddition()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(1, 10, 1.5, -2.5);
      traj2.setCubic(1, 10, 12.5, 3.5);
      assert (traj1.getNumberOfCoefficients() == 4);
      assert (traj2.getNumberOfCoefficients() == 4);
      TrajectoryMathTools.add(traj1, traj1, traj2);
      assert (traj1.getNumberOfCoefficients() == 4);
      assert (traj1.getCoefficient(0) == 13.482853223593963);
      assert (traj1.getCoefficient(1) == 1.069958847736625);
      assert (traj1.getCoefficient(2) == -0.588477366255144);
      assert (traj1.getCoefficient(3) == 0.03566529492455418);
   }

   @Test
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
      assert (traj1.getNumberOfCoefficients() == 4);
      assert (traj2.getNumberOfCoefficients() == 4);
      int numberOfSegments = TrajectoryMathTools.add(resultTrajectoryList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      YoTrajectory traj3 = resultTrajectoryList.get(0);
      YoTrajectory traj4 = resultTrajectoryList.get(1);
      YoTrajectory traj5 = resultTrajectoryList.get(2);
      assert (traj3.getInitialTime() == 1.0);
      assert (traj3.getFinalTime() == 3.0);
      assert (traj3.getNumberOfCoefficients() == 4);
      assert (MathTools.epsilonCompare(traj3.getCoefficient(0), 0.9560, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(1), 1.1520, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(2), -0.6720, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(3), 0.0640, Epsilons.ONE_THOUSANDTH));

      assert (traj4.getInitialTime() == 3.0);
      assert (traj4.getFinalTime() == 5.0);
      assert (traj4.getNumberOfCoefficients() == 4);
      assert (MathTools.epsilonCompare(traj4.getCoefficient(0), 0.9560 - 109.0000, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(1), 1.1520 + 101.2500, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(2), -0.6720 - 27.0000, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(3), 0.0640 + 2.2500, Epsilons.ONE_THOUSANDTH));

      assert (traj5.getInitialTime() == 5.0);
      assert (traj5.getFinalTime() == 6.0);
      assert (traj5.getNumberOfCoefficients() == 4);
      assert (MathTools.epsilonCompare(traj5.getCoefficient(0), 0.9560, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj5.getCoefficient(1), 1.1520, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj5.getCoefficient(2), -0.6720, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj5.getCoefficient(3), 0.0640, Epsilons.ONE_THOUSANDTH));
   }

   @Test
   public void testMultiTimeScaleOperation()
   {
      List<Double> timeList = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0));
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 10, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 10, registry);
      traj1.setConstant(5, 10, 1);
      traj2.setConstant(1, 4, 1);
      int numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 1);
      assert (timeList.get(1) == 4);
      assert (timeList.get(2) == 5);
      assert (timeList.get(3) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(1, 6, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 1);
      assert (timeList.get(1) == 5);
      assert (timeList.get(2) == 6);
      assert (timeList.get(3) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(1, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 1);
      assert (timeList.get(1) == 5);
      assert (timeList.get(2) == 10);
      assert (timeList.get(3) == 11);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(6, 9, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 6);
      assert (timeList.get(2) == 9);
      assert (timeList.get(3) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(6, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 6);
      assert (timeList.get(2) == 10);
      assert (timeList.get(3) == 11);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(11, 12, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 10);
      assert (timeList.get(2) == 11);
      assert (timeList.get(3) == 12);

      // 
      traj2.setConstant(5, 10, 1);
      traj1.setConstant(1, 4, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 1);
      assert (timeList.get(1) == 4);
      assert (timeList.get(2) == 5);
      assert (timeList.get(3) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(1, 6, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 1);
      assert (timeList.get(1) == 5);
      assert (timeList.get(2) == 6);
      assert (timeList.get(3) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(1, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 1);
      assert (timeList.get(1) == 5);
      assert (timeList.get(2) == 10);
      assert (timeList.get(3) == 11);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(6, 9, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 6);
      assert (timeList.get(2) == 9);
      assert (timeList.get(3) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(6, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 6);
      assert (timeList.get(2) == 10);
      assert (timeList.get(3) == 11);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(11, 12, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 10);
      assert (timeList.get(2) == 11);
      assert (timeList.get(3) == 12);

      //
      traj1.setConstant(5, 10, 1);
      traj2.setConstant(5, 9, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 9);
      assert (timeList.get(2) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(5, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 10);
      assert (timeList.get(2) == 11);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(4, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      assert (timeList.get(0) == 4);
      assert (timeList.get(1) == 5);
      assert (timeList.get(2) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(6, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 6);
      assert (timeList.get(2) == 10);

      //
      traj2.setConstant(5, 10, 1);
      traj1.setConstant(5, 9, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 9);
      assert (timeList.get(2) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(5, 11, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 10);
      assert (timeList.get(2) == 11);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(4, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      assert (timeList.get(0) == 4);
      assert (timeList.get(1) == 5);
      assert (timeList.get(2) == 10);

      traj2.setConstant(5, 10, 1);
      traj1.setConstant(6, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 6);
      assert (timeList.get(2) == 10);

      traj1.setConstant(5, 10, 1);
      traj2.setConstant(5, 10, 1);
      numberOfSegments = TrajectoryMathTools.getSegmentTimeList(timeList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 1);
      assert (timeList.get(0) == 5);
      assert (timeList.get(1) == 10);
   }

   @Test
   public void testTrajectorySubtraction()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(0, 2, 3.5, -2.5);
      traj2.setCubic(0, 2, -0.5, -0.6);
      assert (traj1.getNumberOfCoefficients() == 4);
      assert (traj2.getNumberOfCoefficients() == 4);
      TrajectoryMathTools.subtract(traj1, traj1, traj2);
      assert (traj1.getNumberOfCoefficients() == 4);
      assert (MathTools.epsilonCompare(traj1.getCoefficient(0), 3.5 + 0.500, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(1), 0.0 - 0.000, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(2), -4.5 + 0.075, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(3), 1.5 - 0.025, Epsilons.ONE_THOUSANDTH));
   }

   @Test
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
      assert (traj1.getNumberOfCoefficients() == 4);
      assert (traj2.getNumberOfCoefficients() == 4);
      int numberOfSegments = TrajectoryMathTools.subtract(resultTrajectoryList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 3);
      YoTrajectory traj3 = resultTrajectoryList.get(0);
      YoTrajectory traj4 = resultTrajectoryList.get(1);
      YoTrajectory traj5 = resultTrajectoryList.get(2);
      assert (traj3.getInitialTime() == 0.0);
      assert (traj3.getFinalTime() == 1.0);
      assert (traj3.getNumberOfCoefficients() == 4);
      assert (MathTools.epsilonCompare(traj3.getCoefficient(0), 2.1000, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(1), 0.0000, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(2), 0.7000, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(3), -0.0778, Epsilons.ONE_THOUSANDTH));

      assert (traj4.getInitialTime() == 1.0);
      assert (traj4.getFinalTime() == 6.0);
      assert (traj4.getNumberOfCoefficients() == 4);
      assert (MathTools.epsilonCompare(traj4.getCoefficient(0), 2.1000 + 9.9198, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(1), 0.0000 + 1.4074, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(2), 0.7000 - 0.7741, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(3), -0.0778 + 0.0469, Epsilons.ONE_THOUSANDTH));

      assert (traj5.getInitialTime() == 6.0);
      assert (traj5.getFinalTime() == 10.0);
      assert (traj5.getNumberOfCoefficients() == 4);
      assert (MathTools.epsilonCompare(traj5.getCoefficient(0), 9.9198, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj5.getCoefficient(1), 1.4074, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj5.getCoefficient(2), -0.7741, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj5.getCoefficient(3), 0.0469, Epsilons.ONE_THOUSANDTH));
   }

   @Test
   public void testTrajectoryMultiplication()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(12, 15, 19.5, 200.5);
      traj2.setCubic(12, 15, 0.5, 0.1);
      assert (traj1.getNumberOfCoefficients() == 4);
      assert (traj2.getNumberOfCoefficients() == 4);
      TrajectoryMathTools.multiply(traj1, traj1, traj2);
      assert (traj1.getNumberOfCoefficients() == 7);
      assert (MathTools.epsilonCompare(traj1.getCoefficient(0), -2.228097449999825e+06, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(1), 1.016083999999920e+06, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(2), -1.920462999999848e+05, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(3), 1.925763703703550e+04, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(4), -1.080637037036950e+03, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(5), 32.177777777775184, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj1.getCoefficient(6), -0.397256515775002, Epsilons.ONE_THOUSANDTH));
   }

   @Test
   public void testTrajectoryMultiTimeScaleMultiplication()
   {
      List<YoTrajectory> resultTrajectoryList = new ArrayList<>(2);
      resultTrajectoryList.add(new YoTrajectory("ResultTraj0", 7, registry));
      resultTrajectoryList.add(new YoTrajectory("ResultTraj1", 7, registry));
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 7, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 7, registry);
      traj1.setCubic(1, 6, 1.1, 2.5);
      traj2.setCubic(1, 10, -1.6, 0.5);
      assert (traj1.getNumberOfCoefficients() == 4);
      assert (traj2.getNumberOfCoefficients() == 4);
      int numberOfSegments = TrajectoryMathTools.multiply(resultTrajectoryList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      assert (numberOfSegments == 2);
      YoTrajectory traj3 = resultTrajectoryList.get(0);
      YoTrajectory traj4 = resultTrajectoryList.get(1);
      assert (traj3.getInitialTime() == 1.0);
      assert (traj3.getFinalTime() == 6.0);
      assert (traj3.getNumberOfCoefficients() == 7);
      assert (MathTools.epsilonCompare(traj3.getCoefficient(0), -1.956841152263374, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(1), 0.388404938271605, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(2), -0.164315061728395, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(3), -0.052446419753086, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(4), 0.028553086419753, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(5), -0.003484444444444, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj3.getCoefficient(6), 0.000129053497942, Epsilons.ONE_THOUSANDTH));

      assert (traj4.getInitialTime() == 6.0);
      assert (traj4.getFinalTime() == 10.0);
      assert (traj4.getNumberOfCoefficients() == 4);
      assert (MathTools.epsilonCompare(traj4.getCoefficient(0), 0, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(1), 0, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(2), 0, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj4.getCoefficient(3), 0, Epsilons.ONE_THOUSANDTH));
   }

   @Test
   public void test3DTrajectoryAddition()
   {
      YoTrajectory3D traj1 = new YoTrajectory3D("Trajectory1", 3, registry);
      YoTrajectory3D traj2 = new YoTrajectory3D("Trajectory2", 3, registry);
      traj1.setLinear(0, 1, new Point3D(0.5, 0.1, 10), new Point3D(1, 10, 5));
      traj2.setLinear(0, 1, new Point3D(), new Point3D(5, 7.7, 1));
      TrajectoryMathTools.add(traj1, traj1, traj2);

      YoTrajectory traj = traj1.getYoTrajectoryX();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 2);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), 0.5, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), 5.5, Epsilons.ONE_THOUSANDTH));

      traj = traj1.getYoTrajectoryY();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 2);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), 0.1, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), 9.9 + 7.7, Epsilons.ONE_THOUSANDTH));

      traj = traj1.getYoTrajectoryZ();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 2);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), 10, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), -4, Epsilons.ONE_THOUSANDTH));
   }

   @Test
   public void test3DTrajectorySubtraction()
   {
      YoTrajectory3D traj1 = new YoTrajectory3D("Trajectory1", 3, registry);
      YoTrajectory3D traj2 = new YoTrajectory3D("Trajectory2", 3, registry);
      traj1.setLinear(0, 1, new Point3D(0.1, 3.414, 1.87), new Point3D(2.09, 1.35, 5.35));
      traj2.setLinear(0, 1, new Point3D(3.14, 1.59, 12.9), new Point3D(4.51, 5.32, 1.12));
      TrajectoryMathTools.subtract(traj1, traj1, traj2);

      YoTrajectory traj = traj1.getYoTrajectoryX();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 2);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), -3.04, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), 0.62, Epsilons.ONE_THOUSANDTH));

      traj = traj1.getYoTrajectoryY();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 2);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), 1.824, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), -5.794, Epsilons.ONE_THOUSANDTH));

      traj = traj1.getYoTrajectoryZ();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 2);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), -11.03, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), 15.26, Epsilons.ONE_THOUSANDTH));
   }

   @Test
   public void test3DTrajectoryDotProducts()
   {
      YoTrajectory3D traj1 = new YoTrajectory3D("Trajectory1", 3, registry);
      YoTrajectory3D traj2 = new YoTrajectory3D("Trajectory2", 3, registry);
      traj1.setLinear(0, 1, new Point3D(1, 3, 5), new Point3D(6, 4, 2));
      traj2.setLinear(0, 1, new Point3D(2, 4, 6), new Point3D(5, 3, 1));
      TrajectoryMathTools.dotProduct(traj1, traj1, traj2);

      YoTrajectory traj = traj1.getYoTrajectoryX();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), 2, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), 13, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(2), 15, Epsilons.ONE_THOUSANDTH));

      traj = traj1.getYoTrajectoryY();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), 12, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), 1, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(2), -1, Epsilons.ONE_THOUSANDTH));

      traj = traj1.getYoTrajectoryZ();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), 30, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1),-43, Epsilons.ONE_THOUSANDTH));      
      assert (MathTools.epsilonCompare(traj.getCoefficient(2), 15, Epsilons.ONE_THOUSANDTH));
   }

   @Test
   public void test3DTrajectoryCrossProduct()
   {
      YoTrajectory3D traj1 = new YoTrajectory3D("Trajectory1", 3, registry);
      YoTrajectory3D traj2 = new YoTrajectory3D("Trajectory2", 3, registry);
      traj1.setLinear(0, 1, new Point3D(1, 3, 5), new Point3D(6, 4, 2));
      traj2.setLinear(0, 1, new Point3D(2, 4, 6), new Point3D(5, 3, 1));
      TrajectoryMathTools.crossProduct(traj1, traj1, traj2);

      YoTrajectory traj = traj1.getYoTrajectoryX();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), -2, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1),  8, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(2), -8, Epsilons.ONE_THOUSANDTH));

      traj = traj1.getYoTrajectoryY();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0), 4, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), -16, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(2), 16, Epsilons.ONE_THOUSANDTH));

      traj = traj1.getYoTrajectoryZ();
      assert (traj.getInitialTime() == 0);
      assert (traj.getFinalTime() == 1);
      assert (traj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(traj.getCoefficient(0),-2, Epsilons.ONE_THOUSANDTH));
      assert (MathTools.epsilonCompare(traj.getCoefficient(1), 8, Epsilons.ONE_THOUSANDTH));      
      assert (MathTools.epsilonCompare(traj.getCoefficient(2),-8, Epsilons.ONE_THOUSANDTH));      
   }
}
