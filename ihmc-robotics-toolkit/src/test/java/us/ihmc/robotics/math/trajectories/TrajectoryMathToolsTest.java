package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class TrajectoryMathToolsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static TrajectoryMathTools trajMath = new TrajectoryMathTools(16);
   private static final double epsilon = 1e-6;
   private static final int iters = 1000;
   private final Random random = new Random(12903L);

   @Test
   public void testTrajectoryAdditionAndSubtraction()
   {
      int maxNumberOfCoefficients = 10;
      Trajectory trajectory1 = new Trajectory(maxNumberOfCoefficients);
      Trajectory trajectory2 = new Trajectory(maxNumberOfCoefficients);

      int iterations = 50;
      for (int i = 0; i < iterations; i++)
      {
         int numberOfCoefficients = 1 + random.nextInt(maxNumberOfCoefficients);
         double[] trajectory1Coefficients = new double[numberOfCoefficients];
         double[] trajectory2Coefficients = new double[numberOfCoefficients];

         for (int j = 0; j < numberOfCoefficients; j++)
         {
            trajectory1Coefficients[j] = EuclidCoreRandomTools.nextDouble(random, 5.0);
            trajectory2Coefficients[j] = EuclidCoreRandomTools.nextDouble(random, 5.0);
         }

         trajectory1.setDirectly(trajectory1Coefficients);
         trajectory2.setDirectly(trajectory2Coefficients);

         Assert.assertEquals(trajectory1.getNumberOfCoefficients(), numberOfCoefficients);
         Assert.assertEquals(trajectory2.getNumberOfCoefficients(), numberOfCoefficients);

         // test addition
         TrajectoryMathTools.add(trajectory1, trajectory1, trajectory2);
         Assert.assertEquals(trajectory1.getNumberOfCoefficients(), numberOfCoefficients);

         for (int j = 0; j < numberOfCoefficients; j++)
         {
            Assert.assertEquals(trajectory1.getCoefficient(j), trajectory1Coefficients[j] + trajectory2Coefficients[j], epsilon);
         }

         // test subtraction
         trajectory1.setDirectly(trajectory1Coefficients);
         TrajectoryMathTools.subtract(trajectory1, trajectory1, trajectory2);
         Assert.assertEquals(trajectory1.getNumberOfCoefficients(), numberOfCoefficients);

         for (int j = 0; j < numberOfCoefficients; j++)
         {
            Assert.assertEquals(trajectory1.getCoefficient(j), trajectory1Coefficients[j] - trajectory2Coefficients[j], epsilon);
         }
      }
   }

   @Test
   public void testScale()
   {
      Trajectory traj1 = new Trajectory(7);
      Trajectory traj2 = new Trajectory(7);
      Trajectory traj3 = new Trajectory(7);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double startTime = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.1, 100.0);
         double endTime = startTime + duration;

         double scalar = RandomNumbers.nextDouble(random, -10.0, 10.0);

         double x0 = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xd0 = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double xf = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double xdf = RandomNumbers.nextDouble(random, -100.0, 100.0);

         traj1.setCubic(startTime, endTime, x0, xd0, xf, xdf);
         traj2.set(traj1);

         TrajectoryMathTools.scale(traj2, scalar);
         TrajectoryMathTools.scale(traj3, traj1, scalar);

         assertEquals(traj1.getInitialTime(), traj2.getInitialTime(), epsilon);
         assertEquals(traj1.getInitialTime(), traj3.getInitialTime(), epsilon);
         assertEquals(traj1.getFinalTime(), traj2.getFinalTime(), epsilon);
         assertEquals(traj1.getFinalTime(), traj3.getFinalTime(), epsilon);
         double dt = 0.01;
         for (double time = startTime; time < endTime; time += dt)
         {
            traj1.compute(time);
            traj2.compute(time);
            traj3.compute(time);
            assertEquals(scalar * traj1.getPosition(), traj2.getPosition(), epsilon);
            assertEquals(scalar * traj1.getPosition(), traj3.getPosition(), epsilon);
         }
      }

      Trajectory3D traj3d1 = new Trajectory3D(7);
      Trajectory3D traj3d2 = new Trajectory3D(7);
      Trajectory3D traj3d3 = new Trajectory3D(7);

      for (int iter = 0; iter < iters; iter++)
      {
         double startTime = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.1, 100.0);
         double endTime = startTime + duration;

         double scalar = RandomNumbers.nextDouble(random, -10.0, 10.0);

         Point3D x0 = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D xd0 = EuclidCoreRandomTools.nextVector3D(random, -100.0, 100.0);
         Point3D xf = EuclidCoreRandomTools.nextPoint3D(random, -10.0, 10.0);
         Vector3D xdf = EuclidCoreRandomTools.nextVector3D(random, -100.0, 100.0);

         traj3d1.setCubic(startTime, endTime, x0, xd0, xf, xdf);
         traj3d2.set(traj3d1);

         TrajectoryMathTools.scale(scalar, traj3d2);
         TrajectoryMathTools.scale(traj3d3, traj3d1, scalar);

         assertEquals(traj3d1.getInitialTime(), traj3d2.getInitialTime(), epsilon);
         assertEquals(traj3d1.getInitialTime(), traj3d3.getInitialTime(), epsilon);
         assertEquals(traj3d1.getFinalTime(), traj3d2.getFinalTime(), epsilon);
         assertEquals(traj3d1.getFinalTime(), traj3d3.getFinalTime(), epsilon);
         double dt = 0.01;
         for (double time = startTime; time < endTime; time += dt)
         {
            traj3d1.compute(time);
            traj3d2.compute(time);
            traj3d3.compute(time);
            Point3D expectedPosition = new Point3D(traj3d1.getPosition());
            expectedPosition.scale(scalar);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPosition, traj3d2.getPosition(), epsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPosition, traj3d3.getPosition(), epsilon);
         }
      }
   }

   @Test
   public void testTrajectoryMultiTimeScaleAdditionAndSubtraction()
   {
      int maxNumberOfCoefficients = 10;
      int iterations = 25;

      // test addition
      for (int i = 0; i < iterations; i++)
      {
         int numberOfCoefficients = 1 + random.nextInt(maxNumberOfCoefficients);
         testAdditionOrSubtraction(maxNumberOfCoefficients, numberOfCoefficients, true);
      }

      // test subtraction
      for (int i = 0; i < iterations; i++)
      {
         int numberOfCoefficients = 1 + random.nextInt(maxNumberOfCoefficients);
         testAdditionOrSubtraction(maxNumberOfCoefficients, numberOfCoefficients, false);
      }
   }

   private void testAdditionOrSubtraction(int maxNumberOfCoefficients, int numberOfCoefficients, boolean testAddition)
   {
      List<Trajectory> trajectoryList = new ArrayList<>();
      Trajectory trajectory1 = new Trajectory(maxNumberOfCoefficients);
      Trajectory trajectory2 = new Trajectory(maxNumberOfCoefficients);
      TrajectoryMathTools trajectoryMathTools = new TrajectoryMathTools(maxNumberOfCoefficients);

      trajectoryList.clear();
      trajectoryList.add(new Trajectory(numberOfCoefficients));
      trajectoryList.add(new Trajectory(numberOfCoefficients));
      trajectoryList.add(new Trajectory(numberOfCoefficients));

      double[] trajectory1Coefficients = new double[numberOfCoefficients];
      double[] trajectory2Coefficients = new double[numberOfCoefficients];

      double trajectory1StartTime = EuclidCoreRandomTools.nextDouble(random, 5.0);
      double trajectory2StartTime = EuclidCoreRandomTools.nextDouble(random, 5.0);
      double trajectory1EndTime = Math.max(trajectory1StartTime, trajectory2StartTime) + EuclidCoreRandomTools.nextDouble(random, 0.01, 5.0) + epsilon;
      double trajectory2EndTime = Math.max(trajectory1StartTime, trajectory2StartTime) + EuclidCoreRandomTools.nextDouble(random, 0.01, 5.0) + epsilon;

      Trajectory firstTrajectoryToStart = trajectory1StartTime < trajectory2StartTime ? trajectory1 : trajectory2;
      Trajectory lastTrajectoryToEnd = trajectory1EndTime > trajectory2EndTime ? trajectory1 : trajectory2;
      double firstStartTime = Math.min(trajectory1StartTime, trajectory2StartTime);
      double secondStartTime = Math.max(trajectory1StartTime, trajectory2StartTime);
      double firstEndTime = Math.min(trajectory1EndTime, trajectory2EndTime);
      double secondEndTime = Math.max(trajectory1EndTime, trajectory2EndTime);

      for (int j = 0; j < numberOfCoefficients; j++)
      {
         trajectory1Coefficients[j] = EuclidCoreRandomTools.nextDouble(random, 5.0);
         trajectory2Coefficients[j] = EuclidCoreRandomTools.nextDouble(random, 5.0);
      }

      trajectory1.setDirectly(trajectory1Coefficients);
      trajectory2.setDirectly(trajectory2Coefficients);

      trajectory1.setTime(trajectory1StartTime, trajectory1EndTime);
      trajectory2.setTime(trajectory2StartTime, trajectory2EndTime);

      if(testAddition)
      {
         trajectoryMathTools.add(trajectoryList, trajectory1, trajectory2, epsilon);
      }
      else
      {
         trajectoryMathTools.subtract(trajectoryList, trajectory1, trajectory2, epsilon);
      }

      Assert.assertEquals(trajectoryList.size(), 3);

      Trajectory firstSummedTrajectory = trajectoryList.get(0);
      Trajectory secondSummedTrajectory = trajectoryList.get(1);
      Trajectory thirdSummedTrajectory = trajectoryList.get(2);
      Assert.assertEquals(firstSummedTrajectory.getNumberOfCoefficients(), numberOfCoefficients);
      Assert.assertEquals(secondSummedTrajectory.getNumberOfCoefficients(), numberOfCoefficients);
      Assert.assertEquals(thirdSummedTrajectory.getNumberOfCoefficients(), numberOfCoefficients);

      // check first trajectory
      Assert.assertEquals(firstSummedTrajectory.getInitialTime(), firstStartTime, epsilon);
      Assert.assertEquals(firstSummedTrajectory.getFinalTime(), secondStartTime, epsilon);
      double sign = !testAddition && firstTrajectoryToStart == trajectory2 ? -1.0 : 1.0;
      for (int j = 0; j < numberOfCoefficients; j++)
      {
         Assert.assertEquals(sign * firstTrajectoryToStart.getCoefficient(j), firstSummedTrajectory.getCoefficient(j), epsilon);
      }

      // check middle trajectory
      Assert.assertEquals(secondSummedTrajectory.getInitialTime(), secondStartTime, epsilon);
      Assert.assertEquals(secondSummedTrajectory.getFinalTime(), firstEndTime, epsilon);
      sign = testAddition ? 1.0 : -1.0;
      for (int j = 0; j < numberOfCoefficients; j++)
      {
         Assert.assertEquals(trajectory1.getCoefficient(j) + sign * trajectory2.getCoefficient(j), secondSummedTrajectory.getCoefficient(j), epsilon);
      }

      // check last trajectory
      Assert.assertEquals(thirdSummedTrajectory.getInitialTime(), firstEndTime, epsilon);
      Assert.assertEquals(thirdSummedTrajectory.getFinalTime(), secondEndTime, epsilon);
      sign = !testAddition && lastTrajectoryToEnd == trajectory2 ? -1.0 : 1.0;
      for (int j = 0; j < numberOfCoefficients; j++)
      {
         Assert.assertEquals(sign * lastTrajectoryToEnd.getCoefficient(j), thirdSummedTrajectory.getCoefficient(j), epsilon);
      }
   }

   @Test
   public void testMultiTimeScaleOperation()
   {
      TDoubleArrayList timeList = new TDoubleArrayList(4);
      Trajectory traj1 = new Trajectory(10);
      Trajectory traj2 = new Trajectory(10);
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

   @Test
   public void testTrajectorySubtraction()
   {
      Trajectory traj1 = new Trajectory(7);
      Trajectory traj2 = new Trajectory(7);
      traj1.setCubic(0, 2, 3.5, -2.5);
      traj2.setCubic(0, 2, -0.5, -0.6);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      TrajectoryMathTools.subtract(traj1, traj1, traj2);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      assertEquals(traj1.getCoefficient(0), 3.5 + 0.500, epsilon);
      assertEquals(traj1.getCoefficient(1), 0.0 - 0.000, epsilon);
      assertEquals(traj1.getCoefficient(2), -4.5 + 0.075, epsilon);
      assertEquals(traj1.getCoefficient(3), 1.5 - 0.025, epsilon);
   }

   @Test
   public void testTrajectoryMultiTimeScaleSubtraction()
   {
      List<Trajectory> resultTrajectoryList = new ArrayList<>(3);
      resultTrajectoryList.add(new Trajectory(6));
      resultTrajectoryList.add(new Trajectory(6));
      resultTrajectoryList.add(new Trajectory(6));
      resultTrajectoryList.add(new Trajectory(6));
      Trajectory traj1 = new Trajectory(7);
      Trajectory traj2 = new Trajectory(7);
      traj1.setCubic(0, 6, 2.1, 10.5);
      traj2.setCubic(1, 10, -10.6, 6.5);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      int numberOfSegments = trajMath.subtract(resultTrajectoryList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 3);
      Trajectory traj3 = resultTrajectoryList.get(0);
      Trajectory traj4 = resultTrajectoryList.get(1);
      Trajectory traj5 = resultTrajectoryList.get(2);
      assertEquals(traj3.getInitialTime(), 0.0, epsilon);
      assertEquals(traj3.getFinalTime(), 1.0, epsilon);
      Assert.assertTrue(traj3.getNumberOfCoefficients() == 4);
      assertEquals(traj3.getCoefficient(0), 2.1000, epsilon);
      assertEquals(traj3.getCoefficient(1), 0.0000, epsilon);
      assertEquals(traj3.getCoefficient(2), 0.7000, epsilon);
      assertEquals(traj3.getCoefficient(3), -0.0777778, epsilon);

      assertEquals(traj4.getInitialTime(), 1.0, epsilon);
      assertEquals(traj4.getFinalTime(), 6.0, epsilon);
      Assert.assertTrue(traj4.getNumberOfCoefficients() == 4);
      assertEquals(traj4.getCoefficient(0), 2.1000 + 9.91975309, epsilon);
      assertEquals(traj4.getCoefficient(1), 0.0000 + 1.407407407, epsilon);
      assertEquals(traj4.getCoefficient(2), 0.7000 - 0.77407407, epsilon);
      assertEquals(traj4.getCoefficient(3), -0.077777778 + 0.0469135802, epsilon);

      assertEquals(traj5.getInitialTime(), 6.0, epsilon);
      assertEquals(traj5.getFinalTime(), 10.0, epsilon);
      Assert.assertTrue(traj5.getNumberOfCoefficients() == 4);
      assertEquals(traj5.getCoefficient(0), 9.91975309, epsilon);
      assertEquals(traj5.getCoefficient(1), 1.407407407, epsilon);
      assertEquals(traj5.getCoefficient(2), -0.77407407, epsilon);
      assertEquals(traj5.getCoefficient(3), 0.0469135802, epsilon);
   }

   @Test
   public void testTrajectoryMultiplication()
   {
      int maxNumberOfCoefficientsPreMultiply = 5;
      int maxNumberOfCoefficientsPostMultiply = 2 * maxNumberOfCoefficientsPreMultiply - 1;
      Trajectory trajectory1 = new Trajectory(maxNumberOfCoefficientsPostMultiply);
      Trajectory trajectory2 = new Trajectory(maxNumberOfCoefficientsPostMultiply);
      TrajectoryMathTools trajectoryMathTools = new TrajectoryMathTools(maxNumberOfCoefficientsPostMultiply);

      int iterations = 50;
      for (int i = 0; i < iterations; i++)
      {
         int numberOfCoefficients1 = 1 + random.nextInt(maxNumberOfCoefficientsPreMultiply);
         int numberOfCoefficients2 = 1 + random.nextInt(maxNumberOfCoefficientsPreMultiply);

         double[] trajectory1Coefficients = new double[numberOfCoefficients1];
         double[] trajectory2Coefficients = new double[numberOfCoefficients2];

         for (int j = 0; j < numberOfCoefficients1; j++)
         {
            trajectory1Coefficients[j] = EuclidCoreRandomTools.nextDouble(random, 5.0);
         }

         for (int j = 0; j < numberOfCoefficients2; j++)
         {
            trajectory2Coefficients[j] = EuclidCoreRandomTools.nextDouble(random, 5.0);
         }

         trajectory1.setDirectly(trajectory1Coefficients);
         trajectory2.setDirectly(trajectory2Coefficients);

         trajectoryMathTools.multiply(trajectory1, trajectory1, trajectory2);
         int numberOfCoefficientsPostMultiply = numberOfCoefficients1 + numberOfCoefficients2 - 1;
         Assert.assertEquals(trajectory1.getNumberOfCoefficients(), numberOfCoefficientsPostMultiply);

         for (int j = 0; j < numberOfCoefficientsPostMultiply; j++)
         {
            double expectedCoefficient = 0.0;
            for (int k = 0; k <= j; k++)
            {
               if(k >= numberOfCoefficients1 || j - k < 0)
                  break;
               else if(j - k >= numberOfCoefficients2)
                  continue;

               expectedCoefficient += trajectory1Coefficients[k] * trajectory2Coefficients[j - k];
            }

            Assert.assertEquals(trajectory1.getCoefficient(j), expectedCoefficient, epsilon);
         }
      }

      for (int i = 0; i < iters; i++)
      {
         Trajectory traj1 = new Trajectory(4);
         Trajectory traj2 = new Trajectory(4);

         double t0 = random.nextDouble();
         double tf = t0 + RandomNumbers.nextDouble(random, 0.25, 1.0);
         traj1.setCubic(t0, tf, random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         traj2.setCubic(t0, tf, random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());

         Trajectory expected = new Trajectory(10);
         trajectoryMathTools.multiply(expected, traj1, traj2);
         Trajectory actual = new Trajectory(10);
         TrajectoryMathTools.multiplyNaive(actual, traj1, traj2);

         for (int j = 0; j < expected.getNumberOfCoefficients(); j++)
         {
            assertEquals(expected.getCoefficient(j), actual.getCoefficient(j), epsilon);
         }
      }
   }

   @Test
   public void testTrajectoryMultiTimeScaleMultiplication()
   {
      List<Trajectory> resultTrajectoryList = new ArrayList<>(2);
      resultTrajectoryList.add(new Trajectory(7));
      resultTrajectoryList.add(new Trajectory(7));
      Trajectory traj1 = new Trajectory(7);
      Trajectory traj2 = new Trajectory(7);
      traj1.setCubic(1, 6, 1.1, 2.5);
      traj2.setCubic(1, 10, -1.6, 0.5);
      Assert.assertTrue(traj1.getNumberOfCoefficients() == 4);
      Assert.assertTrue(traj2.getNumberOfCoefficients() == 4);
      int numberOfSegments = trajMath.multiply(resultTrajectoryList, traj1, traj2, Epsilons.ONE_MILLIONTH);
      Assert.assertTrue(numberOfSegments == 2);
      Trajectory traj3 = resultTrajectoryList.get(0);
      Trajectory traj4 = resultTrajectoryList.get(1);
      assertEquals(traj3.getInitialTime(), 1.0, epsilon);
      assertEquals(traj3.getFinalTime(), 6.0, epsilon);
      Assert.assertTrue(traj3.getNumberOfCoefficients() == 7);
      assertEquals(traj3.getCoefficient(0), -1.956841152263374, epsilon);
      assertEquals(traj3.getCoefficient(1), 0.388404938271605, epsilon);
      assertEquals(traj3.getCoefficient(2), -0.164315061728395, epsilon);
      assertEquals(traj3.getCoefficient(3), -0.052446419753086, epsilon);
      assertEquals(traj3.getCoefficient(4), 0.028553086419753, epsilon);
      assertEquals(traj3.getCoefficient(5), -0.003484444444444, epsilon);
      assertEquals(traj3.getCoefficient(6), 0.000129053497942, epsilon);

      assertEquals(traj4.getInitialTime(), 6.0, epsilon);
      assertEquals(traj4.getFinalTime(), 10.0, epsilon);
      Assert.assertTrue(traj4.getNumberOfCoefficients() == 4);
      assertEquals(traj4.getCoefficient(0), 0, epsilon);
      assertEquals(traj4.getCoefficient(1), 0, epsilon);
      assertEquals(traj4.getCoefficient(2), 0, epsilon);
      assertEquals(traj4.getCoefficient(3), 0, epsilon);
   }

   @Test
   public void test3DTrajectoryAddition()
   {
      Trajectory3D traj1 = new Trajectory3D(3);
      Trajectory3D traj2 = new Trajectory3D(3);
      traj1.setLinear(0, 1, new Point3D(0.5, 0.1, 10), new Point3D(1, 10, 5));
      traj2.setLinear(0, 1, new Point3D(), new Point3D(5, 7.7, 1));
      TrajectoryMathTools.add(traj1, traj1, traj2);

      Trajectory traj = traj1.getTrajectoryX();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), 0.5, epsilon);
      assertEquals(traj.getCoefficient(1), 5.5, epsilon);

      traj = traj1.getTrajectoryY();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), 0.1, epsilon);
      assertEquals(traj.getCoefficient(1), 9.9 + 7.7, epsilon);

      traj = traj1.getTrajectoryZ();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), 10, epsilon);
      assertEquals(traj.getCoefficient(1), -4, epsilon);
   }

   @Test
   public void test3DTrajectorySubtraction()
   {
      Trajectory3D resultingTrajectory = new Trajectory3D(3);
      Trajectory3D traj1 = new Trajectory3D(3);
      Trajectory3D traj2 = new Trajectory3D(3);
      traj1.setLinear(0, 1, new Point3D(0.1, 3.414, 1.87), new Point3D(2.09, 1.35, 5.35));
      traj2.setLinear(0, 1, new Point3D(3.14, 1.59, 12.9), new Point3D(4.51, 5.32, 1.12));
      TrajectoryMathTools.subtract(resultingTrajectory, traj1, traj2);

      Trajectory traj = resultingTrajectory.getTrajectoryX();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), -3.04, epsilon);
      assertEquals(traj.getCoefficient(1), 0.62, epsilon);

      traj = resultingTrajectory.getTrajectoryY();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), 1.824, epsilon);
      assertEquals(traj.getCoefficient(1), -5.794, epsilon);

      traj = resultingTrajectory.getTrajectoryZ();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), -11.03, epsilon);
      assertEquals(traj.getCoefficient(1), 15.26, epsilon);
   }

   @Test
   public void test3DTrajectorySubtractionEquals()
   {
      Trajectory3D traj1 = new Trajectory3D(3);
      Trajectory3D traj2 = new Trajectory3D(3);
      traj1.setLinear(0, 1, new Point3D(0.1, 3.414, 1.87), new Point3D(2.09, 1.35, 5.35));
      traj2.setLinear(0, 1, new Point3D(3.14, 1.59, 12.9), new Point3D(4.51, 5.32, 1.12));
      TrajectoryMathTools.subtractEquals(traj1, traj2);

      Trajectory traj = traj1.getTrajectoryX();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), -3.04, epsilon);
      assertEquals(traj.getCoefficient(1), 0.62, epsilon);

      traj = traj1.getTrajectoryY();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), 1.824, epsilon);
      assertEquals(traj.getCoefficient(1), -5.794, epsilon);

      traj = traj1.getTrajectoryZ();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 2);
      assertEquals(traj.getCoefficient(0), -11.03, epsilon);
      assertEquals(traj.getCoefficient(1), 15.26, epsilon);
   }

   @Test
   public void test3DTrajectoryDotProducts()
   {
      Trajectory3D traj1 = new Trajectory3D(3);
      Trajectory3D traj2 = new Trajectory3D(3);
      traj1.setLinear(0, 1, new Point3D(1, 3, 5), new Point3D(6, 4, 2));
      traj2.setLinear(0, 1, new Point3D(2, 4, 6), new Point3D(5, 3, 1));
      trajMath.dotProduct(traj1, traj1, traj2);

      Trajectory traj = traj1.getTrajectoryX();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), 2, epsilon);
      assertEquals(traj.getCoefficient(1), 13, epsilon);
      assertEquals(traj.getCoefficient(2), 15, epsilon);

      traj = traj1.getTrajectoryY();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), 12, epsilon);
      assertEquals(traj.getCoefficient(1), 1, epsilon);
      assertEquals(traj.getCoefficient(2), -1, epsilon);

      traj = traj1.getTrajectoryZ();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), 30, epsilon);
      assertEquals(traj.getCoefficient(1), -43, epsilon);
      assertEquals(traj.getCoefficient(2), 15, epsilon);
   }

   @Test
   public void test3DTrajectoryCrossProduct()
   {
      Trajectory3D traj1 = new Trajectory3D(3);
      Trajectory3D traj2 = new Trajectory3D(3);
      Trajectory3D resultingTrajectory = new Trajectory3D(3);
      traj1.setLinear(0, 1, new Point3D(1, 3, 5), new Point3D(6, 4, 2));
      traj2.setLinear(0, 1, new Point3D(2, 4, 6), new Point3D(5, 3, 1));
      trajMath.crossProduct(resultingTrajectory, traj1, traj2);

      Trajectory traj = resultingTrajectory.getTrajectoryX();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), -2, epsilon);
      assertEquals(traj.getCoefficient(1), 8, epsilon);
      assertEquals(traj.getCoefficient(2), -8, epsilon);

      traj = resultingTrajectory.getTrajectoryY();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), 4, epsilon);
      assertEquals(traj.getCoefficient(1), -16, epsilon);
      assertEquals(traj.getCoefficient(2), 16, epsilon);

      traj = resultingTrajectory.getTrajectoryZ();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), -2, epsilon);
      assertEquals(traj.getCoefficient(1), 8, epsilon);
      assertEquals(traj.getCoefficient(2), -8, epsilon);
   }

   @Test
   public void test3DTrajectoryCrossProductStoreInSelf()
   {
      Trajectory3D traj1 = new Trajectory3D(3);
      Trajectory3D traj2 = new Trajectory3D(3);
      traj1.setLinear(0, 1, new Point3D(1, 3, 5), new Point3D(6, 4, 2));
      traj2.setLinear(0, 1, new Point3D(2, 4, 6), new Point3D(5, 3, 1));
      trajMath.crossProduct(traj1, traj2);

      Trajectory traj = traj1.getTrajectoryX();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), -2, epsilon);
      assertEquals(traj.getCoefficient(1), 8, epsilon);
      assertEquals(traj.getCoefficient(2), -8, epsilon);

      traj = traj1.getTrajectoryY();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), 4, epsilon);
      assertEquals(traj.getCoefficient(1), -16, epsilon);
      assertEquals(traj.getCoefficient(2), 16, epsilon);

      traj = traj1.getTrajectoryZ();
      assertEquals(traj.getInitialTime(), 0, epsilon);
      assertEquals(traj.getFinalTime(), 1, epsilon);
      Assert.assertTrue(traj.getNumberOfCoefficients() == 3);
      assertEquals(traj.getCoefficient(0), -2, epsilon);
      assertEquals(traj.getCoefficient(1), 8, epsilon);
      assertEquals(traj.getCoefficient(2), -8, epsilon);
   }

   @Test
   public void testIntegration()
   {
      Trajectory traj1 = new Trajectory(2);
      Trajectory traj2 = new Trajectory(3);
      traj1.setLinear(1, 11, 4, 5);
      TrajectoryMathTools.getIntegral(traj2, traj1);
      assertEquals(traj2.getCoefficient(0), -4.00, epsilon);
      assertEquals(traj2.getCoefficient(1), 3.90, epsilon);
      assertEquals(traj2.getCoefficient(2), 0.05, epsilon);
   }

   @Test
   public void testSimpleDerivative()
   {
      Trajectory cubicTrajectory = new Trajectory(4);
      double[] coefficients = new double[]{-4.0, 3.0, -2.0, 0.5};
      cubicTrajectory.setDirectly(coefficients);
      cubicTrajectory.setTime(1.0, 2.0);

      // check valid derivative call
      Trajectory derivativeTrajectory = new Trajectory(3);
      TrajectoryMathTools.getDerivative(derivativeTrajectory, cubicTrajectory);

      Assert.assertEquals(derivativeTrajectory.getInitialTime(), 1.0, epsilon);
      Assert.assertEquals(derivativeTrajectory.getFinalTime(), 2.0, epsilon);
      Assert.assertEquals(derivativeTrajectory.getNumberOfCoefficients(), 3);
      Assert.assertEquals(derivativeTrajectory.getCoefficient(0), 3.0, epsilon);
      Assert.assertEquals(derivativeTrajectory.getCoefficient(1), -4.0, epsilon);
      Assert.assertEquals(derivativeTrajectory.getCoefficient(2), 1.5, epsilon);

      // try taking derivative without enough coefficients
      try
      {
         TrajectoryMathTools.getDerivative(new Trajectory(2), cubicTrajectory);
         Assert.fail();
      }
      catch(Exception e)
      {
      }
   }

   @Test
   public void testDerivativeOfConstantTrajectory()
   {
      Trajectory constantTrajectory = new Trajectory(1);
      Trajectory derivativeTrajectory = new Trajectory(1);

      constantTrajectory.setConstant(0.0, 1.0, 2.0);
      TrajectoryMathTools.getDerivative(derivativeTrajectory, constantTrajectory);

      Assert.assertEquals(derivativeTrajectory.getInitialTime(), 0.0, epsilon);
      Assert.assertEquals(derivativeTrajectory.getFinalTime(), 1.0, epsilon);
      Assert.assertEquals(derivativeTrajectory.getNumberOfCoefficients(), 1);
      Assert.assertEquals(derivativeTrajectory.getCoefficient(0), 0.0, epsilon);
   }

   @Test
   public void testDerivativeOnRandomTrajectories()
   {
      int maximumNumberOfCoefficients = 10;
      Trajectory trajectory = new Trajectory(maximumNumberOfCoefficients);
      Trajectory derivative = new Trajectory(maximumNumberOfCoefficients - 1);

      int iterations = 50;
      for (int i = 0; i < iterations; i++)
      {
         int numberOfCoefficients = 2 + random.nextInt(maximumNumberOfCoefficients - 1);
         double[] coefficients = RandomNumbers.nextDoubleArray(random, numberOfCoefficients, 10.0);
         trajectory.setDirectly(coefficients);
         TrajectoryMathTools.getDerivative(derivative, trajectory);

         for (int j = 0; j < numberOfCoefficients - 1; j++)
         {
            Assert.assertEquals(derivative.getCoefficient(j), coefficients[j + 1] * (j + 1), epsilon);
         }
      }
   }

   @Test
   public void testDifferentiation()
   {
      Trajectory baseTrajectory = new Trajectory(3);
      Trajectory trajectoryDerivative = new Trajectory(2);

      baseTrajectory.setDirectlyFast(0, 3.0);
      baseTrajectory.setDirectlyFast(1, 4.0);
      baseTrajectory.setDirectlyFast(2, 5.0);

      TrajectoryMathTools.getDerivative(trajectoryDerivative, baseTrajectory);

      assertEquals(4.0, trajectoryDerivative.getCoefficient(0), epsilon);
      assertEquals(10.0, trajectoryDerivative.getCoefficient(1), epsilon);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double startTtime = RandomNumbers.nextDouble(random, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.01, 1000.0);

         double x0 = RandomNumbers.nextDouble(random, 10.0);
         double xd0 = RandomNumbers.nextDouble(random, 100.0);
         double xf = RandomNumbers.nextDouble(random, 10.0);
         double xdf = RandomNumbers.nextDouble(random, 100.0);

         baseTrajectory.setQuadratic(startTtime, startTtime + duration, x0, xd0, xf);

         TrajectoryMathTools.getDerivative(trajectoryDerivative, baseTrajectory);

         assertEquals(baseTrajectory.getCoefficient(1), trajectoryDerivative.getCoefficient(0), epsilon);
         assertEquals(2.0 * baseTrajectory.getCoefficient(2), trajectoryDerivative.getCoefficient(1), epsilon);
      }


      baseTrajectory = new Trajectory(5);
      trajectoryDerivative = new Trajectory(4);

      baseTrajectory.setDirectlyFast(0, 4.0);
      baseTrajectory.setDirectlyFast(1, 5.0);
      baseTrajectory.setDirectlyFast(2, 6.0);
      baseTrajectory.setDirectlyFast(3, 7.0);
      baseTrajectory.setDirectlyFast(4, 8.0);

      TrajectoryMathTools.getDerivative(trajectoryDerivative, baseTrajectory);

      assertEquals(5.0, trajectoryDerivative.getCoefficient(0), epsilon);
      assertEquals(12.0, trajectoryDerivative.getCoefficient(1), epsilon);
      assertEquals(21.0, trajectoryDerivative.getCoefficient(2), epsilon);
      assertEquals(32.0, trajectoryDerivative.getCoefficient(3), epsilon);

      for (int iter = 0; iter < iters; iter++)
      {
         double startTime = RandomNumbers.nextDouble(random, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.01, 1000.0);

         double x0 = RandomNumbers.nextDouble(random, 10.0);
         double xd0 = RandomNumbers.nextDouble(random, 100.0);
         double xdd0 = RandomNumbers.nextDouble(random, 1000.0);
         double xf = RandomNumbers.nextDouble(random, 10.0);
         double xdf = RandomNumbers.nextDouble(random, 100.0);

         baseTrajectory.setQuartic(startTime, startTime + duration, x0, xd0, xdd0, xf, xdf);

         TrajectoryMathTools.getDerivative(trajectoryDerivative, baseTrajectory);

         assertEquals(baseTrajectory.getCoefficient(1), trajectoryDerivative.getCoefficient(0), epsilon);
         assertEquals(2.0 * baseTrajectory.getCoefficient(2), trajectoryDerivative.getCoefficient(1), epsilon);
         assertEquals(3.0 * baseTrajectory.getCoefficient(3), trajectoryDerivative.getCoefficient(2), epsilon);
         assertEquals(4.0 * baseTrajectory.getCoefficient(4), trajectoryDerivative.getCoefficient(3), epsilon);
      }
   }

   @Test
   public void testShifting()
   {
      Trajectory traj1 = new Trajectory(3);
      traj1.setDirectlyFast(0, 1);
      traj1.setDirectlyFast(1, 2);
      traj1.setDirectlyFast(2, 3);
      traj1.setTime(2, 4);
      trajMath.addTimeOffset(traj1, 2);
      assertEquals(traj1.getCoefficient(0), 9, epsilon);
      assertEquals(traj1.getCoefficient(1), -10, epsilon);
      assertEquals(traj1.getCoefficient(2), 3, epsilon);
      trajMath.addTimeOffset(traj1, -4);
      assertEquals(traj1.getCoefficient(0), 17, epsilon);
      assertEquals(traj1.getCoefficient(1), 14, epsilon);
      assertEquals(traj1.getCoefficient(2), 3, epsilon);
   }

   @Test
   public void testSegmentedAddition()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj3 = new SegmentedFrameTrajectory3D(10, 2);
      assertEquals(0, traj1.getNumberOfSegments());
      assertEquals(0, traj2.getNumberOfSegments());
      assertEquals(0, traj3.getNumberOfSegments());

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(2).getFinalTime(), epsilon);

      traj2.add().setLinear(0.5, 0.6, new FramePoint3D(worldFrame, 1, 2, 3), new FramePoint3D(worldFrame, 3, 2, 1));
      traj2.add().setLinear(1.2, 2.2, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));

      assertEquals(0.5, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.6, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.2, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.2, traj2.getSegment(1).getFinalTime(), epsilon);

      // should have 0.0-0.5, 0.5-0.6, 0.6-1.0, 1.0-1.2, 1.2-2.0, 2.0-2.2, 2.2-3.0
      TrajectoryMathTools.addSegmentedTrajectories(traj3, traj1, traj2, Epsilons.ONE_BILLIONTH);

      assertEquals(0.0, traj3.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.5, traj3.getSegment(0).getFinalTime(), epsilon);
      assertEquals(0.5, traj3.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.6, traj3.getSegment(1).getFinalTime(), epsilon);
      assertEquals(0.6, traj3.getSegment(2).getInitialTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(2).getFinalTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(3).getInitialTime(), epsilon);
      assertEquals(1.2, traj3.getSegment(3).getFinalTime(), epsilon);

      assertEquals(1.2, traj3.getSegment(4).getInitialTime(), epsilon);
      assertEquals(2.0, traj3.getSegment(4).getFinalTime(), epsilon);

      assertEquals(2.0, traj3.getSegment(5).getInitialTime(), epsilon);
      assertEquals(2.2, traj3.getSegment(5).getFinalTime(), epsilon);

      assertEquals(2.2, traj3.getSegment(6).getInitialTime(), epsilon);
      assertEquals(3.0, traj3.getSegment(6).getFinalTime(), epsilon);

      assertEquals(7, traj3.getNumberOfSegments());

   }

   @Test
   public void testSegmentedAdditionHard()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj3 = new SegmentedFrameTrajectory3D(20, 2);
      assertEquals(0, traj1.getNumberOfSegments());
      assertEquals(0, traj2.getNumberOfSegments());
      assertEquals(0, traj3.getNumberOfSegments());

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));
      traj1.add().setLinear(3.0, 4.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(4.0, traj1.getSegment(3).getFinalTime(), epsilon);

      traj2.add().setLinear(0.5, 0.6, new FramePoint3D(worldFrame, 1, 2, 3), new FramePoint3D(worldFrame, 3, 2, 1));
      traj2.add().setLinear(0.7, 1.0, new FramePoint3D(worldFrame, 1, 2, 3), new FramePoint3D(worldFrame, 3, 2, 1));
      traj2.add().setLinear(1.2, 2.2, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));
      traj2.add().setLinear(2.7, 3.5, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));
      traj2.add().setLinear(3.9, 4.7, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));

      assertEquals(0.5, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.6, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(0.7, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(1.2, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.2, traj2.getSegment(2).getFinalTime(), epsilon);

      TrajectoryMathTools.addSegmentedTrajectories(traj3, traj1, traj2, Epsilons.ONE_BILLIONTH);

      // should have 0.0-0.5, 0.5-0.6, 0.6-0.7, 0.7-1.0, 1.0-1.2, 1.2-2.0, 2.0-2.2, 2.2-2.7,
      // 2.7-3.0, 3.0-3.5, 3.5-3.9, 3.9-4.0, 4.0-4.7

      assertEquals(0.0, traj3.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.5, traj3.getSegment(0).getFinalTime(), epsilon);

      assertEquals(0.5, traj3.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.6, traj3.getSegment(1).getFinalTime(), epsilon);

      assertEquals(0.6, traj3.getSegment(2).getInitialTime(), epsilon);
      assertEquals(0.7, traj3.getSegment(2).getFinalTime(), epsilon);

      assertEquals(0.7, traj3.getSegment(3).getInitialTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(3).getFinalTime(), epsilon);

      assertEquals(1.0, traj3.getSegment(4).getInitialTime(), epsilon);
      assertEquals(1.2, traj3.getSegment(4).getFinalTime(), epsilon);

      assertEquals(1.2, traj3.getSegment(5).getInitialTime(), epsilon);
      assertEquals(2.0, traj3.getSegment(5).getFinalTime(), epsilon);

      assertEquals(2.0, traj3.getSegment(6).getInitialTime(), epsilon);
      assertEquals(2.2, traj3.getSegment(6).getFinalTime(), epsilon);

      assertEquals(2.2, traj3.getSegment(7).getInitialTime(), epsilon);
      assertEquals(2.7, traj3.getSegment(7).getFinalTime(), epsilon);

      assertEquals(2.7, traj3.getSegment(8).getInitialTime(), epsilon);
      assertEquals(3.0, traj3.getSegment(8).getFinalTime(), epsilon);

      assertEquals(3.0, traj3.getSegment(9).getInitialTime(), epsilon);
      assertEquals(3.5, traj3.getSegment(9).getFinalTime(), epsilon);

      assertEquals(3.5, traj3.getSegment(10).getInitialTime(), epsilon);
      assertEquals(3.9, traj3.getSegment(10).getFinalTime(), epsilon);

      assertEquals(3.9, traj3.getSegment(11).getInitialTime(), epsilon);
      assertEquals(4.0, traj3.getSegment(11).getFinalTime(), epsilon);

      assertEquals(4.0, traj3.getSegment(12).getInitialTime(), epsilon);
      assertEquals(4.7, traj3.getSegment(12).getFinalTime(), epsilon);

      assertEquals(13, traj3.getNumberOfSegments());
   }

   @Test
   public void testSegmentedAdditionWithLimitsA()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj3 = new SegmentedFrameTrajectory3D(10, 2);
      assertEquals(0, traj1.getNumberOfSegments());
      assertEquals(0, traj2.getNumberOfSegments());
      assertEquals(0, traj3.getNumberOfSegments());

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj2.add().setLinear(0.5, 0.6, new FramePoint3D(worldFrame, 1, 2, 3), new FramePoint3D(worldFrame, 3, 2, 1));
      traj2.add().setLinear(1.1, 2.2, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));

      double minimumDuration = 0.11;
      TrajectoryMathTools.addSegmentedTrajectories(traj3, traj1, traj2, minimumDuration, Epsilons.ONE_BILLIONTH);

      assertEquals(0.0, traj3.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.5, traj3.getSegment(0).getFinalTime(), epsilon);

      assertEquals(0.5, traj3.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.6, traj3.getSegment(1).getFinalTime(), epsilon);

      assertEquals(0.6, traj3.getSegment(2).getInitialTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(2).getFinalTime(), epsilon);

      assertEquals(1.0, traj3.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.0, traj3.getSegment(3).getFinalTime(), epsilon);

      assertEquals(2.0, traj3.getSegment(4).getInitialTime(), epsilon);
      assertEquals(2.2, traj3.getSegment(4).getFinalTime(), epsilon);

      assertEquals(2.2, traj3.getSegment(5).getInitialTime(), epsilon);
      assertEquals(3.0, traj3.getSegment(5).getFinalTime(), epsilon);

   }

   @Test
   public void testSegmentedAdditionWithLimitsB()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj3 = new SegmentedFrameTrajectory3D(10, 2);
      assertEquals(0, traj1.getNumberOfSegments());
      assertEquals(0, traj2.getNumberOfSegments());
      assertEquals(0, traj3.getNumberOfSegments());

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj2.add().setLinear(0.5, 0.6, new FramePoint3D(worldFrame, 1, 2, 3), new FramePoint3D(worldFrame, 3, 2, 1));
      traj2.add().setLinear(1.2, 2.1, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));

      double minimumDuration = 0.11;
      TrajectoryMathTools.addSegmentedTrajectories(traj3, traj1, traj2, minimumDuration, Epsilons.ONE_BILLIONTH);

      assertEquals(0.0, traj3.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.5, traj3.getSegment(0).getFinalTime(), epsilon);

      assertEquals(0.5, traj3.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.6, traj3.getSegment(1).getFinalTime(), epsilon);

      assertEquals(0.6, traj3.getSegment(2).getInitialTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(2).getFinalTime(), epsilon);

      assertEquals(1.0, traj3.getSegment(3).getInitialTime(), epsilon);
      assertEquals(1.2, traj3.getSegment(3).getFinalTime(), epsilon);

      assertEquals(1.2, traj3.getSegment(4).getInitialTime(), epsilon);
      assertEquals(2.1, traj3.getSegment(4).getFinalTime(), epsilon);

      assertEquals(2.1, traj3.getSegment(5).getInitialTime(), epsilon);
      assertEquals(3.0, traj3.getSegment(5).getFinalTime(), epsilon);

   }

   @Disabled
   @Test
   public void testSegmentedAdditionWithLimitsC()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj3 = new SegmentedFrameTrajectory3D(10, 2);

      traj1.add().setLinear(0.0, 0.5, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(0.5, 1.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));

      traj2.add().setLinear(0.0, 0.7, new FramePoint3D(worldFrame, 1, 2, 3), new FramePoint3D(worldFrame, 3, 2, 1));
      traj2.add().setLinear(0.7, 0.9, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));

      double minimumDuration = 0.11;
      TrajectoryMathTools.addSegmentedTrajectories(traj3, traj1, traj2, minimumDuration, Epsilons.ONE_BILLIONTH);

      assertEquals(0.0, traj3.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.5, traj3.getSegment(0).getFinalTime(), epsilon);

      assertEquals(0.5, traj3.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.7, traj3.getSegment(1).getFinalTime(), epsilon);

      assertEquals(0.7, traj3.getSegment(2).getInitialTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(2).getFinalTime(), epsilon);
   }

   @Disabled
   @Test
   public void testSegmentedAdditionWithLimitsD()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj3 = new SegmentedFrameTrajectory3D(10, 2);

      traj1.add().setLinear(0.0, 0.5, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(0.5, 1.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));

      traj2.add().setLinear(0.1, 0.7, new FramePoint3D(worldFrame, 1, 2, 3), new FramePoint3D(worldFrame, 3, 2, 1));
      traj2.add().setLinear(0.7, 0.9, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));

      double minimumDuration = 0.11;
      TrajectoryMathTools.addSegmentedTrajectories(traj3, traj1, traj2, minimumDuration, Epsilons.ONE_BILLIONTH);

      assertEquals(0.0, traj3.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.5, traj3.getSegment(0).getFinalTime(), epsilon);

      assertEquals(0.5, traj3.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.7, traj3.getSegment(1).getFinalTime(), epsilon);

      assertEquals(0.7, traj3.getSegment(2).getInitialTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(2).getFinalTime(), epsilon);
   }

   @Test
   public void testSegmentedAdditionWithLimitsE()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj3 = new SegmentedFrameTrajectory3D(10, 2);

      traj1.add().setLinear(0.0, 0.5, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(0.5, 0.9, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));

      traj2.add().setLinear(0.1, 0.7, new FramePoint3D(worldFrame, 1, 2, 3), new FramePoint3D(worldFrame, 3, 2, 1));
      traj2.add().setLinear(0.7, 1.0, new FramePoint3D(worldFrame, 3, 2, 1), new FramePoint3D(worldFrame, 4, 5, 6));

      double minimumDuration = 0.11;
      TrajectoryMathTools.addSegmentedTrajectories(traj3, traj1, traj2, minimumDuration, Epsilons.ONE_BILLIONTH);

      assertEquals(0.0, traj3.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.5, traj3.getSegment(0).getFinalTime(), epsilon);

      assertEquals(0.5, traj3.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.7, traj3.getSegment(1).getFinalTime(), epsilon);

      assertEquals(0.7, traj3.getSegment(2).getInitialTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(2).getFinalTime(), epsilon);
   }

   @Test
   public void testSetCurrentSegmentPolynomial()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj3 = new SegmentedFrameTrajectory3D(10, 2);
      assertEquals(0, traj1.getNumberOfSegments());
      assertEquals(0, traj2.getNumberOfSegments());
      assertEquals(0, traj3.getNumberOfSegments());

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(2).getFinalTime(), epsilon);

      TrajectoryMathTools.setCurrentSegmentPolynomial(traj3.add(), traj1.getSegment(0), 0.0, 1.0, Epsilons.ONE_MILLIONTH);

      assertEquals(1, traj3.getNumberOfSegments());
      assertEquals(0.0, traj3.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj3.getSegment(0).getFinalTime(), epsilon);

      TrajectoryMathTools.setCurrentSegmentPolynomial(traj3.add(), traj1.getSegment(1), 1.0, 2.0, Epsilons.ONE_MILLIONTH);

      assertEquals(2, traj3.getNumberOfSegments());
      assertEquals(1.0, traj3.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj3.getSegment(1).getFinalTime(), epsilon);

      TrajectoryMathTools.setCurrentSegmentPolynomial(traj3.add(), traj1.getSegment(2), 2.25, 2.5, Epsilons.ONE_MILLIONTH);

      assertEquals(3, traj3.getNumberOfSegments());
      assertEquals(2.250, traj3.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.5, traj3.getSegment(2).getFinalTime(), epsilon);
   }


   @Test
   public void testRemoveShortSegments()
   {
      SegmentedFrameTrajectory3D traj = new SegmentedFrameTrajectory3D(4, 2);

      traj.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.removeShortSegments(traj, 0.11);

      assertEquals(3, traj.getNumberOfSegments());
      assertEquals(0.0, traj.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj.getSegment(2).getInitialTime(), epsilon);
      assertEquals(3.0, traj.getSegment(2).getFinalTime(), epsilon);

      traj.reset();

      traj.add().setLinear(0.0, 0.1, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(2.0, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.removeShortSegments(traj, 0.11);

      assertEquals(3, traj.getNumberOfSegments());
      assertEquals(0.0, traj.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj.getSegment(2).getInitialTime(), epsilon);
      assertEquals(3.0, traj.getSegment(2).getFinalTime(), epsilon);

      traj.reset();

      traj.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(2.0, 2.9, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(2.9, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.removeShortSegments(traj, 0.11);

      assertEquals(3, traj.getNumberOfSegments());
      assertEquals(0.0, traj.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj.getSegment(2).getInitialTime(), epsilon);
      assertEquals(3.0, traj.getSegment(2).getFinalTime(), epsilon);



      traj.reset();

      traj.add().setLinear(0.0, 0.1, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj.add().setLinear(0.1, 0.5, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(0.5, 0.6, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(0.6, 0.7, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj.add().setLinear(0.7, 1.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));
      traj.add().setLinear(1.0, 1.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));
      traj.add().setLinear(1.9, 2.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));
      traj.add().setLinear(2.0, 2.2, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));
      traj.add().setLinear(2.2, 2.3, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));
      traj.add().setLinear(2.3, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.removeShortSegments(traj, 0.11);

      assertEquals(6, traj.getNumberOfSegments());
      assertEquals(0.0, traj.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.5, traj.getSegment(0).getFinalTime(), epsilon);
      assertEquals(0.5, traj.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.7, traj.getSegment(1).getFinalTime(), epsilon);
      assertEquals(0.7, traj.getSegment(2).getInitialTime(), epsilon);
      assertEquals(1.0, traj.getSegment(2).getFinalTime(), epsilon);
      assertEquals(1.0, traj.getSegment(3).getInitialTime(), epsilon);
      assertEquals(1.9, traj.getSegment(3).getFinalTime(), epsilon);
      assertEquals(1.9, traj.getSegment(4).getInitialTime(), epsilon);
      assertEquals(2.3, traj.getSegment(4).getFinalTime(), epsilon);
      assertEquals(2.3, traj.getSegment(5).getInitialTime(), epsilon);
      assertEquals(3.0, traj.getSegment(5).getFinalTime(), epsilon);

   }


   @Test
   public void testStretchTrajectoryToMatchBounds()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj2.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.stretchTrajectoriesToMatchBounds(traj1, traj2);

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(3).getFinalTime(), epsilon);

      assertEquals(0.0, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(3.0, traj2.getSegment(3).getFinalTime(), epsilon);


      traj1.reset();
      traj2.reset();

      traj2.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.stretchTrajectoriesToMatchBounds(traj1, traj2);

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(3).getFinalTime(), epsilon);

      assertEquals(0.0, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(3.0, traj2.getSegment(3).getFinalTime(), epsilon);


      traj1.reset();
      traj2.reset();

      traj2.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.stretchTrajectoriesToMatchBounds(traj1, traj2);

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(3).getFinalTime(), epsilon);

      assertEquals(0.0, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(3.0, traj2.getSegment(3).getFinalTime(), epsilon);


      traj1.reset();
      traj2.reset();

      traj2.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.stretchTrajectoriesToMatchBounds(traj1, traj2);

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(3).getFinalTime(), epsilon);

      assertEquals(0.0, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(3.0, traj2.getSegment(3).getFinalTime(), epsilon);
   }

   @Test
   public void testShrinkTrajectoryToMatchBounds()
   {
      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 2);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 2);

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj2.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.shrinkTrajectoriesToMatchBounds(traj1, traj2);

      assertEquals(0.1, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.9, traj1.getSegment(3).getFinalTime(), epsilon);

      assertEquals(0.1, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.9, traj2.getSegment(3).getFinalTime(), epsilon);


      traj1.reset();
      traj2.reset();

      traj2.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.shrinkTrajectoriesToMatchBounds(traj1, traj2);

      assertEquals(0.1, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.9, traj1.getSegment(3).getFinalTime(), epsilon);

      assertEquals(0.1, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.9, traj2.getSegment(3).getFinalTime(), epsilon);


      traj1.reset();
      traj2.reset();

      traj2.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.shrinkTrajectoriesToMatchBounds(traj1, traj2);

      assertEquals(0.1, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.9, traj1.getSegment(3).getFinalTime(), epsilon);

      assertEquals(0.1, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.9, traj2.getSegment(3).getFinalTime(), epsilon);


      traj1.reset();
      traj2.reset();

      traj2.add().setLinear(0.0, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.0, 2.0, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.0, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj1.add().setLinear(2.1, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      TrajectoryMathTools.shrinkTrajectoriesToMatchBounds(traj1, traj2);

      assertEquals(0.1, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.9, traj1.getSegment(3).getFinalTime(), epsilon);

      assertEquals(0.1, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(2.0, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.9, traj2.getSegment(3).getFinalTime(), epsilon);
   }


   @Test
   public void testResampleTrajectoryToMatchWaypoints()
   {
      TrajectoryMathTools tools = new TrajectoryMathTools(10);

      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 5);
      SegmentedFrameTrajectory3D traj1Desired = new SegmentedFrameTrajectory3D(4, 5);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 5);

      traj1.add().setLinear(0.0, 1.5, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setLinear(1.5, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1Desired.setAll(traj1);

      traj2.add().setLinear(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setLinear(1.0, 1.4, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(1.4, 1.6, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(1.6, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setLinear(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      tools.resampleTrajectoryToMatchWaypoints(traj1, traj2, 0.11);

      FramePoint3D expectedPosition = new FramePoint3D();
      FrameVector3D expectedVelocity = new FrameVector3D();
      FrameVector3D expectedAcceleration = new FrameVector3D();

      FramePoint3D actualPosition = new FramePoint3D();
      FrameVector3D actualVelocity = new FrameVector3D();
      FrameVector3D actualAcceleration = new FrameVector3D();

      double[] timesToTest = new double[]{0.0, 1.4, 3.0};

      for (double time : timesToTest)
      {
         traj1Desired.update(time, expectedPosition, expectedVelocity, expectedAcceleration);
         traj1.update(time, actualPosition, actualVelocity, actualAcceleration);

         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedPosition, actualPosition, epsilon);
      }

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.4, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.4, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(1).getFinalTime(), epsilon);

      assertEquals(0.0, traj2.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.0, traj2.getSegment(1).getInitialTime(), epsilon);
      assertEquals(1.4, traj2.getSegment(1).getFinalTime(), epsilon);
      assertEquals(1.4, traj2.getSegment(2).getInitialTime(), epsilon);
      assertEquals(1.6, traj2.getSegment(2).getFinalTime(), epsilon);
      assertEquals(1.6, traj2.getSegment(3).getInitialTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(3).getFinalTime(), epsilon);
      assertEquals(2.1, traj2.getSegment(4).getInitialTime(), epsilon);
      assertEquals(3.0, traj2.getSegment(4).getFinalTime(), epsilon);



      traj1.reset();
      traj1Desired.reset();
      traj2.reset();

      traj1.add().setCubic(0.0, 1.5, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setCubic(1.5, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1Desired.setAll(traj1);

      traj2.add().setCubic(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setCubic(1.0, 1.4, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(1.4, 1.6, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(1.6, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      tools.resampleTrajectoryToMatchWaypoints(traj1, traj2, 0.11);

      expectedPosition = new FramePoint3D();
      expectedVelocity = new FrameVector3D();
      expectedAcceleration = new FrameVector3D();

      actualPosition = new FramePoint3D();
      actualVelocity = new FrameVector3D();
      actualAcceleration = new FrameVector3D();

      timesToTest = new double[]{0.0, 1.4, 3.0};

      for (double time : timesToTest)
      {
         traj1Desired.update(time, expectedPosition, expectedVelocity, expectedAcceleration);
         traj1.update(time, actualPosition, actualVelocity, actualAcceleration);

         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedPosition, actualPosition, epsilon);
         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedVelocity, actualVelocity, epsilon);
      }

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.4, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.4, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(1).getFinalTime(), epsilon);






      traj1.reset();
      traj1Desired.reset();
      traj2.reset();

      traj1.add().setCubic(0.0, 1.5, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setCubic(1.5, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1Desired.setAll(traj1);

      traj2.add().setCubic(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setCubic(1.0, 1.3, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(1.4, 1.6, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(1.6, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      tools.resampleTrajectoryToMatchWaypoints(traj1, traj2, 0.11);

      expectedPosition = new FramePoint3D();
      expectedVelocity = new FrameVector3D();
      expectedAcceleration = new FrameVector3D();

      actualPosition = new FramePoint3D();
      actualVelocity = new FrameVector3D();
      actualAcceleration = new FrameVector3D();


      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.6, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.6, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(1).getFinalTime(), epsilon);

      timesToTest = new double[]{0.0, 1.6, 3.0};

      for (double time : timesToTest)
      {
         traj1Desired.update(time, expectedPosition, expectedVelocity, expectedAcceleration);
         traj1.update(time, actualPosition, actualVelocity, actualAcceleration);

         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedPosition, actualPosition, epsilon);
         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedVelocity, actualVelocity, epsilon);
      }




      traj1.reset();
      traj1Desired.reset();
      traj2.reset();

      traj1.add().setCubic(0.0, 1.5, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj1.add().setCubic(1.5, 3.0, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      traj1Desired.setAll(traj1);

      traj2.add().setCubic(0.1, 1.0, new FramePoint3D(worldFrame, 10, 11, 12), new FramePoint3D(worldFrame, 13, 14, 15));
      traj2.add().setCubic(1.0, 1.3, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(1.4, 1.7, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(1.6, 2.1, new FramePoint3D(worldFrame, 15, 20, 25), new FramePoint3D(worldFrame, 20, 25, 30));
      traj2.add().setCubic(2.1, 2.9, new FramePoint3D(worldFrame, 25, 28, 31), new FramePoint3D(worldFrame, 35, 38, 41));

      tools.resampleTrajectoryToMatchWaypoints(traj1, traj2, 0.11);

      expectedPosition = new FramePoint3D();
      expectedVelocity = new FrameVector3D();
      expectedAcceleration = new FrameVector3D();

      actualPosition = new FramePoint3D();
      actualVelocity = new FrameVector3D();
      actualAcceleration = new FrameVector3D();


      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(1.5, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(1.5, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(3.0, traj1.getSegment(1).getFinalTime(), epsilon);

      timesToTest = new double[]{0.0, 1.5, 3.0};

      for (double time : timesToTest)
      {
         traj1Desired.update(time, expectedPosition, expectedVelocity, expectedAcceleration);
         traj1.update(time, actualPosition, actualVelocity, actualAcceleration);

         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedPosition, actualPosition, epsilon);
         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedVelocity, actualVelocity, epsilon);
      }


   }

   @Test
   public void testResampleTrajectoryToMatchWaypointsHard()
   {
      TrajectoryMathTools tools = new TrajectoryMathTools(10);

      SegmentedFrameTrajectory3D traj1 = new SegmentedFrameTrajectory3D(4, 5);
      SegmentedFrameTrajectory3D traj1Desired = new SegmentedFrameTrajectory3D(4, 5);
      SegmentedFrameTrajectory3D traj2 = new SegmentedFrameTrajectory3D(4, 5);

      Random random = new Random(1738L);

      traj1.add().setCubic(0.0, 0.25,  EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0,10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj1.add().setCubic(0.25, 0.5,  EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0,10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj1.add().setCubic(0.5, 0.75,  EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0,10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj1.add().setCubic(0.75, 1.0,  EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0,10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj1.add().setCubic(1.0, 1.25,  EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0,10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj1.add().setCubic(1.25, 1.50, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0,10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj1.add().setCubic(1.5, 1.75,  EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0,10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj1.add().setCubic(1.75, 2.0,  EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0,10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));

      traj1Desired.setAll(traj1);

      traj2.add().setCubic(0.0, 0.13, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj2.add().setCubic(0.13, 0.26, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj2.add().setCubic(0.26, 0.4, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj2.add().setCubic(0.4, 0.52, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj2.add().setCubic(0.52, 1.01, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj2.add().setCubic(1.01, 1.05, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj2.add().setCubic(1.05, 1.26, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj2.add().setCubic(1.26, 1.48, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
      traj2.add().setCubic(1.48, 1.9, EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0), EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));

      tools.resampleTrajectoryToMatchWaypoints(traj1, traj2, 0.11);

      FramePoint3D expectedPosition = new FramePoint3D();
      FrameVector3D expectedVelocity = new FrameVector3D();
      FrameVector3D expectedAcceleration = new FrameVector3D();

      FramePoint3D actualPosition = new FramePoint3D();
      FrameVector3D actualVelocity = new FrameVector3D();
      FrameVector3D actualAcceleration = new FrameVector3D();

      assertEquals(0.0, traj1.getSegment(0).getInitialTime(), epsilon);
      assertEquals(0.26, traj1.getSegment(0).getFinalTime(), epsilon);
      assertEquals(0.26, traj1.getSegment(1).getInitialTime(), epsilon);
      assertEquals(0.52, traj1.getSegment(1).getFinalTime(), epsilon);
      assertEquals(0.52, traj1.getSegment(2).getInitialTime(), epsilon);
      assertEquals(0.75, traj1.getSegment(2).getFinalTime(), epsilon);
      assertEquals(0.75, traj1.getSegment(3).getInitialTime(), epsilon);
      assertEquals(1.01, traj1.getSegment(3).getFinalTime(), epsilon);
      assertEquals(1.01, traj1.getSegment(4).getInitialTime(), epsilon);
      assertEquals(1.26, traj1.getSegment(4).getFinalTime(), epsilon);
      assertEquals(1.26, traj1.getSegment(5).getInitialTime(), epsilon);
      assertEquals(1.48, traj1.getSegment(5).getFinalTime(), epsilon);
      assertEquals(1.48, traj1.getSegment(6).getInitialTime(), epsilon);
      assertEquals(1.75, traj1.getSegment(6).getFinalTime(), epsilon);
      assertEquals(1.75, traj1.getSegment(7).getInitialTime(), epsilon);
      assertEquals(2.0, traj1.getSegment(7).getFinalTime(), epsilon);

      double[] timesToTest = new double[]{0.0, 0.26, 0.52, 0.75, 1.01, 1.26, 1.48, 1.75, 2.0};

      for (double time : timesToTest)
      {
         traj1Desired.update(time, expectedPosition, expectedVelocity, expectedAcceleration);
         traj1.update(time, actualPosition, actualVelocity, actualAcceleration);

         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedPosition, actualPosition, epsilon);
         EuclidCoreTestTools.assertTuple3DEquals("Failed at time " + time + ".", expectedVelocity, actualVelocity, epsilon);
      }


   }
}
