package us.ihmc.perception.detections;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;

import java.lang.reflect.InvocationTargetException;
import java.time.Duration;
import java.time.Instant;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

@SuppressWarnings("CallToPrintStackTrace")
public class DetectionManagerTest
{
   private static final Random random = new Random(0);
   private static final AtomicBoolean testPassed = new AtomicBoolean(true);

   @Test
   public void testAddDetections() throws InvocationTargetException, NoSuchMethodException, InstantiationException, IllegalAccessException
   {
      DetectionManager detectionManager = new DetectionManager();

      // Generate test detection sets
      Set<InstantDetection> testDetectionsA = generateDetectionFrame(3);
      Set<InstantDetection> testDetectionsB = generateDetectionFrame(3);

      // add the detection sets to detection manager
      assertDoesNotThrow(() -> detectionManager.addDetections(testDetectionsA, InstantDetection.class));
      assertDoesNotThrow(() -> detectionManager.addDetections(testDetectionsB, InstantDetection.class));

      // check whether detection manager received the sets properly
      Set<PersistentDetection> storedDetectionsA = detectionManager.getDetectionsOfType(InstantDetection.class);
      assertEquals(3, storedDetectionsA.size());
      for (PersistentDetection persistentDetection : storedDetectionsA)
      {
         assertNotNull(persistentDetection.getMostRecentDetection());
         assertTrue(testDetectionsA.contains(persistentDetection.getMostRecentDetection()));
      }

      Set<PersistentDetection> storedDetectionsB = detectionManager.getDetectionsOfType(InstantDetection.class);
      assertEquals(3, storedDetectionsB.size());
      for (PersistentDetection persistentDetection : storedDetectionsB)
      {
         assertNotNull(persistentDetection.getMostRecentDetection());
         assertTrue(testDetectionsB.contains(persistentDetection.getMostRecentDetection()));
      }
   }

   @Test
   public void testMatchingDetections() throws InvocationTargetException, NoSuchMethodException, InstantiationException, IllegalAccessException
   {
      DetectionManager detectionManager = new DetectionManager();

      // Generate the first frame of detections & add to detection manager
      Set<InstantDetection> firstFrame = generateDetectionFrame(3);
      detectionManager.addDetections(firstFrame, InstantDetection.class);

      // Generate second frame of detections & add to detection manager
      Set<InstantDetection> secondFrame = generateDetectionFrame(2);
      detectionManager.addDetections(secondFrame, InstantDetection.class);

      // Ensure detection manager has 3 detections
      Set<PersistentDetection> persistentDetections = detectionManager.getDetectionsOfType(InstantDetection.class);
      assertEquals(3, persistentDetections.size());

      // Ensure detection manager matched only 2 detections, and left one old detection
      int numContained = 0;
      for (PersistentDetection persistentDetection : persistentDetections)
      {
         if (secondFrame.contains(persistentDetection.getMostRecentDetection()))
            numContained++;
         else
            assertTrue(firstFrame.contains(persistentDetection.getMostRecentDetection()));
      }
      assertEquals(2, numContained);
   }

   @Test
   public void testConcurrentDetectionAddition()
         throws InvocationTargetException, NoSuchMethodException, InstantiationException, IllegalAccessException, InterruptedException
   {
      testPassed.set(true);
      DetectionManager detectionManager = new DetectionManager();

      int numRuns = 1000; // If test is passing when it really shouldn't, try increasing these numbers. Ensure numRuns >= maxDetections
      int maxDetections = 1000;

      // Repeat test numerous times to catch any random race condition
      for (int i = 1; i <= numRuns; ++i)
      {
         int numToGenerate = i % (maxDetections + 1);

         // Two threads attempt to add different classes of detections concurrently. This should not throw exceptions.
         Set<InstantDetection> detectionFrameA = generateDetectionFrame(numToGenerate);
         Thread threadA = new Thread(() ->
         {
            ThreadTools.sleep(random.nextInt(10));
            try
            {
               detectionManager.addDetections(detectionFrameA, InstantDetection.class);
            }
            catch (Exception e)
            {
               e.printStackTrace();
               testPassed.set(false);
            }
         }, "TestThreadA");

         Set<InstantDetection> detectionFrameB = generateDetectionFrame(2 * numToGenerate);
         Thread threadB = new Thread (() ->
         {
            ThreadTools.sleep(random.nextInt(10));
            try
            {
               detectionManager.addDetections(detectionFrameB, InstantDetection.class);
            }
            catch (Exception e)
            {
               e.printStackTrace();
               testPassed.set(false);
            }
         }, "TestThreadB");

         threadA.start();
         threadB.start();

         threadA.join();
         threadB.join();

         assertTrue(testPassed.get());
      }

      // Number of detections in detection manager should be correct after all runs
      Set<PersistentDetection> detectionsA = detectionManager.getDetectionsOfType(InstantDetection.class);
      assertEquals(maxDetections, detectionsA.size());

      Set<PersistentDetection> detectionsB = detectionManager.getDetectionsOfType(InstantDetection.class);
      assertEquals(2 * maxDetections, detectionsB.size());
   }

   /**
    * Two threads add detections of different classes while the main thread accesses the added detections
    */
   @Test
   public void testConcurrentAdditionAndAccess()
   {
      testPassed.set(true);

      DetectionManager detectionManager = new DetectionManager();
      detectionManager.setDefaultHistoryDuration(Duration.ofDays(5));
      detectionManager.setMatchDistanceThreshold(10.0);

      // Create the two addition threads
      final int numRuns = 5000;
      Thread additionThreadA = new Thread(() ->
      {
         for (int i = 0; i < numRuns && testPassed.get(); ++i)
         {
            Set<InstantDetection> detectionsA = generateDetectionFrame(100);
            detectionManager.addDetections(detectionsA, InstantDetection.class);
         }
      }, "AdditionThreadA");

      Thread additionThreadB = new Thread(() ->
      {
         for (int i = 0; i < numRuns && testPassed.get(); ++i)
         {
            Set<InstantDetection> detectionsB = generateDetectionFrame(100);
            detectionManager.addDetections(detectionsB, InstantDetection.class);
         }
      }, "AdditionThreadB");

      additionThreadA.start();
      additionThreadB.start();

      boolean receivedDetectionsA = false;
      boolean receivedDetectionsB = false;
      // While the addition threads are running, attempt to access and update data
      while (testPassed.get() && (additionThreadA.isAlive() || additionThreadB.isAlive()))
      {
         detectionManager.updateDetections();

         Set<PersistentDetection> detectionsA = detectionManager.getDetectionsOfType(InstantDetection.class);
         Set<PersistentDetection> detectionsB = detectionManager.getDetectionsOfType(InstantDetection.class);

         // Once detections are received, there should be 100 of them.
         if (!detectionsA.isEmpty() || receivedDetectionsA)
         {
            receivedDetectionsA = true;
            assertEquals(100, detectionsA.size());
         }
         if (!detectionsB.isEmpty() || receivedDetectionsB)
         {
            receivedDetectionsB = true;
            assertEquals(100, detectionsB.size());
         }

         detectionsA.forEach(detection ->
         {
            assertEquals(InstantDetection.class, detection.getInstantDetectionClass());
            assertTrue(detection.getDetectedObjectClass().contains(InstantDetection.class.getSimpleName()));
         });

         detectionsB.forEach(detection ->
         {
            assertEquals(InstantDetection.class, detection.getInstantDetectionClass());
            assertTrue(detection.getDetectedObjectClass().contains(InstantDetection.class.getSimpleName()));
         });

         ThreadTools.sleep(10);
      }

      try
      {
         additionThreadA.join();
         additionThreadB.join();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
         testPassed.set(false);
      }

      assertTrue(testPassed.get());
   }

   @Test
   public void testDetectionStability() throws InvocationTargetException, NoSuchMethodException, InstantiationException, IllegalAccessException
   {
      DetectionManager detectionManager = new DetectionManager();
      detectionManager.setMatchDistanceThreshold(0.5);
      detectionManager.setDefaultStabilityThreshold(0.5);
      detectionManager.setDefaultStabilityFrequency(0.9);
      detectionManager.setDefaultHistorySeconds(2.5);

      Instant startTime = Instant.now();

      for (int i = 0; i < 5; ++i)
      {
         Set<InstantDetection> testDetectionsA = generateDetectionFrame(5, startTime.minusSeconds(i));
         detectionManager.addDetections(testDetectionsA, InstantDetection.class);

         Set<InstantDetection> testDetectionsB = generateDetectionFrame(5, startTime.minusSeconds(i));
         detectionManager.addDetections(testDetectionsB, InstantDetection.class);
      }

      detectionManager.updateDetections(startTime);

      Set<PersistentDetection> persistentDetectionsA = detectionManager.getDetectionsOfType(InstantDetection.class);
      for (PersistentDetection persistentDetection : persistentDetectionsA)
      {
         assertFalse(persistentDetection.isStable());
         assertEquals(3, persistentDetection.getHistorySize());
      }

      Set<PersistentDetection> persistentDetectionsB = detectionManager.getDetectionsOfType(InstantDetection.class);
      for (PersistentDetection persistentDetection : persistentDetectionsB)
      {
         assertTrue(persistentDetection.isStable());
         assertEquals(3, persistentDetection.getHistorySize());
      }

      Instant future = startTime.plusSeconds(3);
      detectionManager.updateDetections(future);

      for (PersistentDetection persistentDetection : persistentDetectionsA)
      {
         assertFalse(persistentDetection.isStable());
         assertEquals(1, persistentDetection.getHistorySize());
      }

      for (PersistentDetection persistentDetection : persistentDetectionsB)
      {
         assertFalse(persistentDetection.isStable());
         assertEquals(1, persistentDetection.getHistorySize());
      }
   }

   public static Set<InstantDetection> generateDetectionFrame(int numberToGenerate)
   {
      return generateDetectionFrame(numberToGenerate, Instant.now());
   }

   public static Set<InstantDetection> generateDetectionFrame(int numberToGenerate, Instant now)
   {
      Set<InstantDetection> testDetections = new HashSet<>();

      for (int i = 0; i < numberToGenerate; ++i)
      {
         InstantDetection testDetection  = new InstantDetection("detection_" + i, 1.0, new Pose3D(), now);
         testDetections.add(testDetection);
      }

      return testDetections;
   }
}
