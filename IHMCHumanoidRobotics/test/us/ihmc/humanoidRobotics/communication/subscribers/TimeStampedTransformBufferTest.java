package us.ihmc.humanoidRobotics.communication.subscribers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.kinematics.TransformInterpolationCalculator;

public class TimeStampedTransformBufferTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEmptyBuffer()
   {
      Random random = new Random(5616L);

      TimeStampedTransformBuffer timeStampedPelvisPoseBuffer = new TimeStampedTransformBuffer(5151);

      assertEquals(timeStampedPelvisPoseBuffer.getNewestTimestamp(), 0L);
      assertEquals(timeStampedPelvisPoseBuffer.getOldestTimestamp(), Long.MAX_VALUE);

      for (int i = 0; i < 1000; i++)
         assertFalse(timeStampedPelvisPoseBuffer.isInRange(random.nextLong()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testThatPosesAreProperlyStored()
   {
      int bufferSize = 100;
      Random random = new Random(5616L);

      TimeStampedTransformBuffer timeStampedPelvisPoseBuffer = new TimeStampedTransformBuffer(bufferSize);
      TimeStampedTransform3D toTest = new TimeStampedTransform3D();

      ArrayList<TimeStampedTransform3D> pelvisPosesRegistered = new ArrayList<>(bufferSize);
      long timeStamp = 0L;
      long expectedNewestTimeStamp = 0L;
      long expectedOldestTimeStamp = 0L;

      for (int i = 0; i < bufferSize; i++)
      {
         RigidBodyTransform newestPelvisPose = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         timeStamp += (long) RandomNumbers.nextInt(random, 1, 1516);
         TimeStampedTransform3D newTimeStampedTransform = new TimeStampedTransform3D(newestPelvisPose, timeStamp);
         pelvisPosesRegistered.add(newTimeStampedTransform);
         timeStampedPelvisPoseBuffer.put(newestPelvisPose, timeStamp);

         if (i == 0)
            expectedOldestTimeStamp = timeStamp;
      }

      expectedNewestTimeStamp = timeStamp;

      assertEquals(timeStampedPelvisPoseBuffer.getNewestTimestamp(), expectedNewestTimeStamp);
      assertEquals(timeStampedPelvisPoseBuffer.getOldestTimestamp(), expectedOldestTimeStamp);

      for (int i = 0; i < bufferSize; i++)
      {
         TimeStampedTransform3D expected = pelvisPosesRegistered.get(i);
         timeStampedPelvisPoseBuffer.findTransform(expected.getTimeStamp(), toTest);

         assertTrue(expected.epsilonEquals(toTest, 1.0e-10));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testThatPosesAreProperlyStoredEvenAfterFillingTheBuffer()
   {
      int bufferSize = 100;
      Random random = new Random(5616L);

      TimeStampedTransformBuffer timeStampedPelvisPoseBuffer = new TimeStampedTransformBuffer(bufferSize);
      TimeStampedTransform3D toTest = new TimeStampedTransform3D();

      ArrayList<TimeStampedTransform3D> pelvisPosesRegistered = new ArrayList<>(bufferSize);
      long timeStamp = 0L;
      long expectedNewestTimeStamp = 0L;

      for (int i = 0; i < 100; i++)
      {
         int numberOfPoses = RandomNumbers.nextInt(random, 1, bufferSize);
         pelvisPosesRegistered.clear();

         for (int poseIndex = 0; poseIndex < numberOfPoses; poseIndex++)
         {
            RigidBodyTransform newestPelvisPose = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
            timeStamp += (long) RandomNumbers.nextInt(random, 1, 1516);
            TimeStampedTransform3D newTimeStampedTransform = new TimeStampedTransform3D(newestPelvisPose, timeStamp);
            pelvisPosesRegistered.add(newTimeStampedTransform);
            timeStampedPelvisPoseBuffer.put(newestPelvisPose, timeStamp);
         }

         expectedNewestTimeStamp = timeStamp;

         assertEquals(timeStampedPelvisPoseBuffer.getNewestTimestamp(), expectedNewestTimeStamp);

         for (int poseIndex = 0; poseIndex < numberOfPoses; poseIndex++)
         {
            TimeStampedTransform3D expected = pelvisPosesRegistered.get(poseIndex);
            timeStampedPelvisPoseBuffer.findTransform(expected.getTimeStamp(), toTest);

            assertTrue(expected.epsilonEquals(toTest, 1.0e-10));
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testInterpolationBetweenPoses()
   {
      int bufferSize = 100;
      Random random = new Random(5616L);

      TimeStampedTransformBuffer timeStampedPelvisPoseBuffer = new TimeStampedTransformBuffer(bufferSize);
      TimeStampedTransform3D toTest = new TimeStampedTransform3D();
      TimeStampedTransform3D expected = new TimeStampedTransform3D();

      TransformInterpolationCalculator transformInterpolationCalculator = new TransformInterpolationCalculator();

      ArrayList<TimeStampedTransform3D> pelvisPosesRegistered = new ArrayList<>(bufferSize);
      long timeStamp = 0L;
      long expectedNewestTimeStamp = 0L;

      int numberOfPoses = bufferSize;
      pelvisPosesRegistered.clear();

      for (int poseIndex = 0; poseIndex < numberOfPoses; poseIndex++)
      {
         RigidBodyTransform newestPelvisPose = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         timeStamp += (long) RandomNumbers.nextInt(random, 1, 1516);
         TimeStampedTransform3D newTimeStampedTransform = new TimeStampedTransform3D(newestPelvisPose, timeStamp);
         pelvisPosesRegistered.add(newTimeStampedTransform);
         timeStampedPelvisPoseBuffer.put(newestPelvisPose, timeStamp);
      }

      expectedNewestTimeStamp = timeStamp;

      assertEquals(timeStampedPelvisPoseBuffer.getNewestTimestamp(), expectedNewestTimeStamp);

      for (int poseIndex = 0; poseIndex < numberOfPoses - 1; poseIndex++)
      {
         TimeStampedTransform3D previous = pelvisPosesRegistered.get(poseIndex);
         TimeStampedTransform3D next = pelvisPosesRegistered.get(poseIndex + 1);

         for (long currentTimeStamp = previous.getTimeStamp(); currentTimeStamp <= next.getTimeStamp(); currentTimeStamp++)
         {
            transformInterpolationCalculator.interpolate(previous, next, expected, currentTimeStamp);
            timeStampedPelvisPoseBuffer.findTransform(currentTimeStamp, toTest);

            assertTrue(expected.epsilonEquals(toTest, 1.0e-10));
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testInterpolationBetweenPosesAfterFillingBuffer()
   {
      int bufferSize = 100;
      Random random = new Random(5616L);

      TimeStampedTransformBuffer timeStampedPelvisPoseBuffer = new TimeStampedTransformBuffer(bufferSize);
      TimeStampedTransform3D toTest = new TimeStampedTransform3D();
      TimeStampedTransform3D expected = new TimeStampedTransform3D();

      TransformInterpolationCalculator transformInterpolationCalculator = new TransformInterpolationCalculator();

      ArrayList<TimeStampedTransform3D> pelvisPosesRegistered = new ArrayList<>(bufferSize);
      long timeStamp = 0L;
      long expectedNewestTimeStamp = 0L;

      int numberOfPoses = bufferSize;
      pelvisPosesRegistered.clear();

      for (int i = 0; i < 10; i++)
      {
         pelvisPosesRegistered.clear();
         for (int j = 0; j < RandomNumbers.nextInt(random, 1, bufferSize); j++)
         {
            timeStampedPelvisPoseBuffer.put(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random), timeStamp);
         }

         for (int poseIndex = 0; poseIndex < numberOfPoses; poseIndex++)
         {
            RigidBodyTransform newestPelvisPose = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
            timeStamp += (long) RandomNumbers.nextInt(random, 1, 1516);
            TimeStampedTransform3D newTimeStampedTransform = new TimeStampedTransform3D(newestPelvisPose, timeStamp);
            pelvisPosesRegistered.add(newTimeStampedTransform);
            timeStampedPelvisPoseBuffer.put(newestPelvisPose, timeStamp);
         }

         expectedNewestTimeStamp = timeStamp;

         assertEquals(timeStampedPelvisPoseBuffer.getNewestTimestamp(), expectedNewestTimeStamp);

         for (int poseIndex = 0; poseIndex < numberOfPoses - 1; poseIndex++)
         {
            TimeStampedTransform3D previous = pelvisPosesRegistered.get(poseIndex);
            TimeStampedTransform3D next = pelvisPosesRegistered.get(poseIndex + 1);

            for (long currentTimeStamp = previous.getTimeStamp(); currentTimeStamp <= next.getTimeStamp(); currentTimeStamp++)
            {
               transformInterpolationCalculator.interpolate(previous, next, expected, currentTimeStamp);
               timeStampedPelvisPoseBuffer.findTransform(currentTimeStamp, toTest);

               assertTrue(expected.epsilonEquals(toTest, 1.0e-10));
            }
         }
      }
   }
}
