package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoConcatenatedSplinesTest
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final static double EPSILON = 1;
   private final static Random random = new Random(2468642L);

	@ContinuousIntegrationTest(estimatedDuration = 1.8)
	@Test(timeout = 30000)
   public void testSimpleCubicQuinticCubic()
   {
      int trials = 50;

      for (int trial = 0; trial < trials; trial++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("ConcatenatedSplinesTest");
         double[] times = getRandomTimes(4);
         FramePoint[] positions = getRandomPositions(4);
         FrameVector[] velocities = getRandomVelocities(4);
         
         YoConcatenatedSplines concatenatedSplines = new YoConcatenatedSplines(new int[] {4, 6, 4}, worldFrame, 100, registry, "original");

         concatenatedSplines.setCubicQuinticCubic(times, positions, velocities);

         for (int i = 0; i < 4; i++)
         {
            concatenatedSplines.compute(times[i]);
            FramePoint expectedPosition = positions[i];
            FramePoint actualPosition = concatenatedSplines.getPosition();
            FrameVector expectedVelocity = velocities[i];
            FrameVector actualVelocity = concatenatedSplines.getVelocity();
            for (Direction direction : Direction.values())
            {
               assertEquals(expectedPosition.get(direction), actualPosition.get(direction), EPSILON);
               assertEquals(expectedVelocity.get(direction), actualVelocity.get(direction), EPSILON);
            }
         }
      }
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testSimpleCubicQuinticCubicTroublesome()
   {
      YoVariableRegistry registry = new YoVariableRegistry("ConcatenatedSplinesTest");
      
      FramePoint p0 = new FramePoint(worldFrame, 3.6468620919827135, -4.5527029914004205, -2.7826785633396045);
      FramePoint p1 = new FramePoint(worldFrame, 1.873647248784934, 4.8291937131106835, 3.1811710040983314);
      FramePoint p2 = new FramePoint(worldFrame, 4.804858430375251, 1.0594552970638658, 1.5041612749446536);
      FramePoint p3 = new FramePoint(worldFrame, -3.956325341635919, 1.9215355811412582, -0.6302175630608513);
      
      FrameVector v0 = new FrameVector(worldFrame, -4.8568993877181565, 0.1075944215258593, -1.6784029597793193);
      FrameVector v1 = new FrameVector(worldFrame, 0.7643232534897653, 2.343350060177201, -4.41734069825054);
      FrameVector v2 = new FrameVector(worldFrame, -0.6049982080383529, 0.15877078502014896, 4.057649141458521);
      FrameVector v3 = new FrameVector(worldFrame, -4.96359745963404, 1.6376702749164398, 3.5095804627690903);
      
      double[] times = new double[]{0.6999570397272431, 2.5250057526516496, 2.5267781471484474, 3.681342287856614};
      FramePoint[] positions = new FramePoint[]{p0, p1, p2, p3};
      FrameVector[] velocities = new FrameVector[]{v0, v1, v2, v3};
      
      YoConcatenatedSplines concatenatedSplines = new YoConcatenatedSplines(new int[] {4, 6, 4}, worldFrame, 100, registry, "original");

      concatenatedSplines.setCubicQuinticCubic(times, positions, velocities);

      for (int i = 0; i < 4; i++)
      {
         concatenatedSplines.compute(times[i]);
         FramePoint expectedPosition = positions[i];
         FramePoint actualPosition = concatenatedSplines.getPosition();
         FrameVector expectedVelocity = velocities[i];
         FrameVector actualVelocity = concatenatedSplines.getVelocity();
         for (Direction direction : Direction.values())
         {
            assertEquals(expectedPosition.get(direction), actualPosition.get(direction), EPSILON);
            assertEquals(expectedVelocity.get(direction), actualVelocity.get(direction), EPSILON);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.3)
	@Test(timeout = 30000)
   public void testQuinticsFromCubicQuinticCubic()
   {
      int trials = 50;

      for (int trial = 0; trial < trials; trial++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("ConcatenatedSplinesTest");
         double[] times = getRandomTimes(4);
         FramePoint[] positions = getRandomPositions(4);
         FrameVector[] velocities = getRandomVelocities(4);

         YoConcatenatedSplines originalSplines = new YoConcatenatedSplines(new int[] {4, 6, 4}, worldFrame, 100, registry, "original");
         YoConcatenatedSplines respacedSplines = new YoConcatenatedSplines(new int[] {6, 6, 6, 6}, worldFrame, 100, registry, "respaced");

         originalSplines.setCubicQuinticCubic(times, positions, velocities);

         double[] oldTimes = getRandomTimes(5);
         double[] newTimes = getRandomTimes(5);
         respacedSplines.setQuintics(originalSplines, oldTimes, newTimes);

         for (int i = 0; i < oldTimes.length; i++)
         {
            originalSplines.compute(oldTimes[i]);
            respacedSplines.compute(newTimes[i]);
            FramePoint originalPosition = originalSplines.getPosition();
            FramePoint respacedPosition = respacedSplines.getPosition();
            FrameVector originalVelocity = originalSplines.getVelocity();
            FrameVector respacedVelocity = respacedSplines.getVelocity();
            for (Direction direction : Direction.values())
            {
               assertEquals(originalPosition.get(direction), respacedPosition.get(direction), EPSILON);
               assertEquals(originalVelocity.get(direction), respacedVelocity.get(direction), EPSILON);
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.4)
	@Test(timeout = 30000)
   public void testTimeFromArcLength()
   {
      int trials = 50;

      for (int trial = 0; trial < trials; trial++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("ConcatenatedSplinesTest");
         double[] times = getRandomTimes(4);
         FramePoint[] positions = getRandomPositions(4);
         FrameVector[] velocities = getRandomVelocities(4);

         YoConcatenatedSplines concatenatedSplines = new YoConcatenatedSplines(new int[] {4, 6, 4}, worldFrame, 100, registry, "original");

         concatenatedSplines.setCubicQuinticCubic(times, positions, velocities);

         double arcLengthThroughSecondSpline = concatenatedSplines.getSplineByIndex(0).getArcLength() + concatenatedSplines.getSplineByIndex(1).getArcLength();
         double arcLengthThroughThirdSpline = arcLengthThroughSecondSpline + concatenatedSplines.getSplineByIndex(2).getArcLength();
         double arcLengthSomewhereInBetween = (arcLengthThroughSecondSpline + arcLengthThroughThirdSpline) / 2.0;

         double actualTimeThroughSecondSpline = concatenatedSplines.approximateTimeFromArcLength(arcLengthThroughSecondSpline);
         double actualTimeThroughThirdSpline = concatenatedSplines.approximateTimeFromArcLength(arcLengthThroughThirdSpline);
         double actualTimeInBetween = concatenatedSplines.approximateTimeFromArcLength(arcLengthSomewhereInBetween);

         double expectedTimeThroughSecondSpline = times[2];
         double expectedTimeThroughThirdSpline = times[3];

         assertEquals(expectedTimeThroughSecondSpline, actualTimeThroughSecondSpline, EPSILON);
         assertEquals(expectedTimeThroughThirdSpline, actualTimeThroughThirdSpline, EPSILON);
         assertTrue(expectedTimeThroughSecondSpline <= actualTimeInBetween);
         assertTrue(actualTimeInBetween <= expectedTimeThroughThirdSpline);
      }
   }

   private double[] getRandomTimes(int number)
   {
      double[] times = new double[number];
      for (int i = 0; i < number; i++)
      {
         times[i] = RandomNumbers.nextDouble(random, 0.0, 5.0);
      }

      Arrays.sort(times);
      for (int i = 0; i < times.length - 1; i++)
      {
         if (times[i + 1] - times[i] < .2)
         {
            times[i + 1] = times[i] + .2;
         }
      }

      return times;
   }

   public FramePoint[] getRandomPositions(int number)
   {
      FramePoint[] positions = new FramePoint[number];
      for (int i = 0; i < number; i++)
      {
         positions[i] = new FramePoint(worldFrame);

         for (Direction direction : Direction.values())
         {
            positions[i].set(direction, RandomNumbers.nextDouble(random, -5.0, 5.0));
         }
      }

      return positions;
   }

   private FrameVector[] getRandomVelocities(int number)
   {
      FrameVector[] velocities = new FrameVector[number];
      for (int i = 0; i < number; i++)
      {
         velocities[i] = new FrameVector(worldFrame);

         for (Direction direction : Direction.values())
         {
            velocities[i].set(direction, RandomNumbers.nextDouble(random, -5.0, 5.0));
         }
      }

      return velocities;
   }
}
