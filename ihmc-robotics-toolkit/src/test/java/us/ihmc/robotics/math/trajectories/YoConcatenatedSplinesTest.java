package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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
         FramePoint3D[] positions = getRandomPositions(4);
         FrameVector3D[] velocities = getRandomVelocities(4);
         
         YoConcatenatedSplines concatenatedSplines = new YoConcatenatedSplines(new int[] {4, 6, 4}, worldFrame, 100, registry, "original");

         concatenatedSplines.setCubicQuinticCubic(times, positions, velocities);

         for (int i = 0; i < 4; i++)
         {
            concatenatedSplines.compute(times[i]);
            FramePoint3D expectedPosition = positions[i];
            FramePoint3D actualPosition = concatenatedSplines.getPosition();
            FrameVector3D expectedVelocity = velocities[i];
            FrameVector3D actualVelocity = concatenatedSplines.getVelocity();
            for (Axis axis : Axis.values())
            {
               assertEquals(expectedPosition.getElement(axis.ordinal()), actualPosition.getElement(axis.ordinal()), EPSILON);
               assertEquals(expectedVelocity.getElement(axis.ordinal()), actualVelocity.getElement(axis.ordinal()), EPSILON);
            }
         }
      }
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testSimpleCubicQuinticCubicTroublesome()
   {
      YoVariableRegistry registry = new YoVariableRegistry("ConcatenatedSplinesTest");
      
      FramePoint3D p0 = new FramePoint3D(worldFrame, 3.6468620919827135, -4.5527029914004205, -2.7826785633396045);
      FramePoint3D p1 = new FramePoint3D(worldFrame, 1.873647248784934, 4.8291937131106835, 3.1811710040983314);
      FramePoint3D p2 = new FramePoint3D(worldFrame, 4.804858430375251, 1.0594552970638658, 1.5041612749446536);
      FramePoint3D p3 = new FramePoint3D(worldFrame, -3.956325341635919, 1.9215355811412582, -0.6302175630608513);
      
      FrameVector3D v0 = new FrameVector3D(worldFrame, -4.8568993877181565, 0.1075944215258593, -1.6784029597793193);
      FrameVector3D v1 = new FrameVector3D(worldFrame, 0.7643232534897653, 2.343350060177201, -4.41734069825054);
      FrameVector3D v2 = new FrameVector3D(worldFrame, -0.6049982080383529, 0.15877078502014896, 4.057649141458521);
      FrameVector3D v3 = new FrameVector3D(worldFrame, -4.96359745963404, 1.6376702749164398, 3.5095804627690903);
      
      double[] times = new double[]{0.6999570397272431, 2.5250057526516496, 2.5267781471484474, 3.681342287856614};
      FramePoint3D[] positions = new FramePoint3D[]{p0, p1, p2, p3};
      FrameVector3D[] velocities = new FrameVector3D[]{v0, v1, v2, v3};
      
      YoConcatenatedSplines concatenatedSplines = new YoConcatenatedSplines(new int[] {4, 6, 4}, worldFrame, 100, registry, "original");

      concatenatedSplines.setCubicQuinticCubic(times, positions, velocities);

      for (int i = 0; i < 4; i++)
      {
         concatenatedSplines.compute(times[i]);
         FramePoint3D expectedPosition = positions[i];
         FramePoint3D actualPosition = concatenatedSplines.getPosition();
         FrameVector3D expectedVelocity = velocities[i];
         FrameVector3D actualVelocity = concatenatedSplines.getVelocity();
         for (Axis axis : Axis.values())
         {
            assertEquals(expectedPosition.getElement(axis.ordinal()), actualPosition.getElement(axis.ordinal()), EPSILON);
            assertEquals(expectedVelocity.getElement(axis.ordinal()), actualVelocity.getElement(axis.ordinal()), EPSILON);
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
         FramePoint3D[] positions = getRandomPositions(4);
         FrameVector3D[] velocities = getRandomVelocities(4);

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
            FramePoint3D originalPosition = originalSplines.getPosition();
            FramePoint3D respacedPosition = respacedSplines.getPosition();
            FrameVector3D originalVelocity = originalSplines.getVelocity();
            FrameVector3D respacedVelocity = respacedSplines.getVelocity();
            for (Axis axis : Axis.values())
            {
               assertEquals(originalPosition.getElement(axis.ordinal()), respacedPosition.getElement(axis.ordinal()), EPSILON);
               assertEquals(originalVelocity.getElement(axis.ordinal()), respacedVelocity.getElement(axis.ordinal()), EPSILON);
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
         FramePoint3D[] positions = getRandomPositions(4);
         FrameVector3D[] velocities = getRandomVelocities(4);

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

   public FramePoint3D[] getRandomPositions(int number)
   {
      FramePoint3D[] positions = new FramePoint3D[number];
      for (int i = 0; i < number; i++)
      {
         positions[i] = new FramePoint3D(worldFrame);

         for (Axis axis : Axis.values())
         {
            positions[i].setElement(axis.ordinal(), RandomNumbers.nextDouble(random, -5.0, 5.0));
         }
      }

      return positions;
   }

   private FrameVector3D[] getRandomVelocities(int number)
   {
      FrameVector3D[] velocities = new FrameVector3D[number];
      for (int i = 0; i < number; i++)
      {
         velocities[i] = new FrameVector3D(worldFrame);

         for (Axis axis : Axis.values())
         {
            velocities[i].setElement(axis.ordinal(), RandomNumbers.nextDouble(random, -5.0, 5.0));
         }
      }

      return velocities;
   }
}
