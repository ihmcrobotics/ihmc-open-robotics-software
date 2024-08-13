package us.ihmc.perception.detections;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;

import java.time.Instant;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

public class PersistentDetectionTest
{
   @Test
   public void testGetMostRecentDetection()
   {
      Instant startInstant = Instant.now();

      double confidence = 0.5;
      double historyDuration = 1.0;
      Pose3D detectionPose = new Pose3D();
      InstantDetection mostRecentDetection = new InstantDetection("TestDetection", confidence, detectionPose, startInstant);
      PersistentDetection persistentDetection = new PersistentDetection(mostRecentDetection, 1.0, 0.5, 5.0, historyDuration);
      for (int i = 1; i < 10; ++i)
      {
         confidence = 1.0;
         // go back in time, and add a detection every second for the last 10 seconds.
         InstantDetection testDetection = new InstantDetection("TestDetection", confidence, detectionPose, startInstant.minusSeconds(i));
         assertTrue(testDetection.getDetectionTime().isBefore(mostRecentDetection.getDetectionTime()));
         persistentDetection.addDetection(testDetection);
      }
      // We know that the original detection should still be the first one, since we added only instants in the past.
      assertEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());

      // Check that the history is the expected size, and that it's sorted correctly.
      assertEquals(10, persistentDetection.getHistorySize());
      assertEquals(10, persistentDetection.getDetectionHistory().size());
      for (int i = 0; i < 9; i++)
      {
         List<InstantDetection> detectionList = persistentDetection.getDetectionHistory().stream().toList();
         assertTrue(detectionList.get(i).getDetectionTime().isBefore(detectionList.get(i + 1).getDetectionTime()));
      }

      // So we want to clear out all the old history, but not all of it
      Instant instantInFuture = startInstant.plusSeconds(1);
      persistentDetection.setHistoryDuration(3.0);
      Instant expectedExpirationTime = instantInFuture.minusSeconds(3);

      persistentDetection.updateHistory(instantInFuture);
      assertEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());
      assertTrue(instantInFuture.isAfter(mostRecentDetection.getDetectionTime()));
      assertEquals(3, persistentDetection.getHistorySize());
      // make sure the oldest point isn't before the expiration window.
      assertEquals(persistentDetection.getOldestDetection(), persistentDetection.getOldestDetection());
      assertFalse(persistentDetection.getOldestDetection().getDetectionTime().isBefore(expectedExpirationTime));

      // We want to clear out all the old history, but we don't want to remove the most recent detection.
      instantInFuture = startInstant.plusSeconds(15);
      persistentDetection.updateHistory(instantInFuture);
      assertEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());
      assertEquals(mostRecentDetection, persistentDetection.getOldestDetection());
      assertEquals(1, persistentDetection.getHistorySize());
      assertEquals(startInstant, mostRecentDetection.getDetectionTime());

      for (int i = 1; i < 10; ++i)
      {
         confidence = 1.0;
         // go forward in time, and add a detection every second for the next 10 seconds.
         Instant newInstant = startInstant.plusSeconds(i);
         InstantDetection testDetection = new InstantDetection("TestDetection", confidence, detectionPose, newInstant);
         persistentDetection.addDetection(testDetection);

         assertEquals(startInstant, mostRecentDetection.getDetectionTime());
         assertTrue(newInstant.isAfter(startInstant));
         // We know that each new detection should become the most recent detection, because it occurs after the start instant
         assertNotEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());
         assertEquals(testDetection, persistentDetection.getMostRecentDetection());
      }

      // We know the oldest detection should be the first detection, since all the new detections added were after it. THat means that pruning the history
      // Shouldn't change its length
      int originalSize = persistentDetection.getHistorySize();
      persistentDetection.updateHistory(startInstant);
      assertEquals(originalSize, persistentDetection.getHistorySize());
      assertEquals(startInstant, persistentDetection.getOldestDetection().getDetectionTime());


      // reset to only one detection
      mostRecentDetection = persistentDetection.getMostRecentDetection();
      startInstant = mostRecentDetection.getDetectionTime();
      persistentDetection.setHistoryDuration(0.5);
      persistentDetection.updateHistory(startInstant);
      assertEquals(1, persistentDetection.getHistorySize());


      // do a tiny time delta to make sure we're tolerant to small differences, too
      persistentDetection.addDetection(new InstantDetection("TestDetection", confidence, detectionPose, startInstant.minusNanos(1)));
      assertEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());

      InstantDetection mostRecentDetectionExpected = new InstantDetection("TestDetection", confidence, detectionPose, startInstant.plusNanos(1));
      persistentDetection.addDetection(mostRecentDetectionExpected);
      assertEquals(mostRecentDetectionExpected, persistentDetection.getMostRecentDetection());
   }

   @Test
   public void testAverageConfidence()
   {
      PersistentDetection persistentDetection = null;

      Instant startInstant = Instant.now();
      for (int i = 0; i < 10; ++i)
      {
         InstantDetection testDetection = new InstantDetection("TestDetection", i / 10.0, new Pose3D(), startInstant.minusSeconds(i));
         if (persistentDetection == null)
            persistentDetection = new PersistentDetection(testDetection, 1.0, 0.5, 0.0, 10.0);
         else
            persistentDetection.addDetection(testDetection);
      }

      persistentDetection.updateHistory(startInstant);
      assertEquals(0.45, persistentDetection.getAverageConfidence());
   }

   @Test
   public void testDetectionFrequency()
   {
      PersistentDetection persistentDetection = null;

      Instant startInstant = Instant.now();
      for (int i = 0; i < 10; ++i)
      {
         InstantDetection testDetection = new InstantDetection("TestDetection", 1.0, new Pose3D(), startInstant.plusSeconds(i));
         if (persistentDetection == null)
            persistentDetection = new PersistentDetection(testDetection, 1.0, 0.5, 5.0, 1.0);
         else
            persistentDetection.addDetection(testDetection);
      }

      assertEquals(1.0, persistentDetection.getDetectionFrequency(), 1E-7);
      assertEquals(1.0, persistentDetection.getDetectionFrequencyDecaying(startInstant.plusSeconds(10)), 1E-7);
      assertEquals(0.5, persistentDetection.getDetectionFrequencyDecaying(startInstant.plusSeconds(20)), 1E-7);
   }

   @Test
   public void testIsStableDetection()
   {
      PersistentDetection persistentDetection = null;

      Instant startInstant = Instant.now();
      for (int i = 0; i < 10; ++i)
      {
         InstantDetection testDetection = new InstantDetection("TestDetection", (10.0 - i) / 10.0, new Pose3D(), startInstant.plusSeconds(i));
         if (persistentDetection == null)
            persistentDetection = new PersistentDetection(testDetection, 1.0, 0.4, 0.5, 10.0);
         else
            persistentDetection.addDetection(testDetection);
      }

      Instant future = startInstant.plusSeconds(11);
      assertTrue(persistentDetection.isStable(future));

      persistentDetection.setHistoryDuration(5.0);
      persistentDetection.updateHistory(future);
      assertFalse(persistentDetection.isStable(future));

      persistentDetection.setStabilityConfidenceThreshold(0.19);
      assertTrue(persistentDetection.isStable(future));
   }

   @Test
   public void testExceptions()
   {
      InstantDetection detectionClassA = new InstantDetection("ClassA", 1.0, new Pose3D(), Instant.now());

      PersistentDetection persistentDetection = new PersistentDetection(detectionClassA, 1.0, 0.5, 5.0, 1.0);

      // Don't allow a negative history length
      assertThrows(IllegalArgumentException.class, () -> persistentDetection.setHistoryDuration(-1.0));

      // Don't allow bad arguments in construction
      assertThrows(IllegalArgumentException.class, () -> new PersistentDetection(detectionClassA, 1.0, 1.0, 0.0, -0.5));
   }

   @Test
   public void testConcurrentModificationAndAccess() throws InterruptedException
   {
      Random random = new Random(0L);

      String objectClass = "testObject";
      double confidence = 0.5;
      Pose3D detectionPose = new Pose3D();
      InstantDetection firstInstantDetection = new InstantDetection(objectClass, confidence, detectionPose, Instant.now());

      double poseFilterAlpha = 0.5;
      double stabilityConfidence = 0.5;
      double stabilityDetectionFrequency = 30.0;
      double historyDuration = 1.0;
      PersistentDetection persistentDetection = new PersistentDetection(firstInstantDetection,
                                                                        poseFilterAlpha,
                                                                        stabilityConfidence,
                                                                        stabilityDetectionFrequency,
                                                                        historyDuration);

      final AtomicBoolean testPassing = new AtomicBoolean(true);
      Thread detectionProducer = new Thread(() ->
      {
         for (int i = 0; i < 1000 && testPassing.get(); ++i)
         {
            try
            {
               int sleepMillis = random.nextInt(10);
               ThreadTools.sleep(sleepMillis);
               InstantDetection newDetection = new InstantDetection(objectClass, confidence, detectionPose, Instant.now());
               persistentDetection.addDetection(newDetection);
            }
            catch (ConcurrentModificationException concurrentModificationException)
            {
               testPassing.set(false);
               LogTools.error("ConcurrentModificationException thrown in producer.");
               concurrentModificationException.printStackTrace();
            }
            catch (Exception exception)
            {
               testPassing.set(false);
               LogTools.error("Exception thrown in producer. Message: \n" + exception.getMessage());
               exception.printStackTrace();
            }
         }
      }, "TestDetectionProducer");

      Thread detectionConsumer = new Thread(() ->
      {
         while (detectionProducer.isAlive() && testPassing.get())
         {
            int sleepMillis = random.nextInt(10);
            ThreadTools.sleep(sleepMillis);

            try
            {
               persistentDetection.updateHistory(Instant.now());
               persistentDetection.getAverageConfidence();
               persistentDetection.getDetectionFrequency();
               persistentDetection.getDetectionFrequencyDecaying(Instant.now());
               persistentDetection.isStable();
            }
            catch (ConcurrentModificationException concurrentModificationException)
            {
               testPassing.set(false);
               LogTools.error("ConcurrentModificationException thrown in consumer. \n");
               concurrentModificationException.printStackTrace();
            }
            catch (Exception exception)
            {
               testPassing.set(false);
               LogTools.error("Exception thrown in consumer.");
               exception.printStackTrace();
            }
         }
      }, "TestDetectionConsumer");

      detectionProducer.start();
      detectionConsumer.start();

      detectionProducer.join();
      detectionConsumer.join();

      assertTrue(testPassing.get());
   }
}
