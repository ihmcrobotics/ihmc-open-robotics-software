package us.ihmc.perception.detections;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

public class DetectionManagerTest
{
   private static final Random random = new Random(0);
   private static final AtomicBoolean testPassed = new AtomicBoolean(true);

   @Test
   public void testAddDetections()
   {
      DetectionManager detectionManager = new DetectionManager(null);

      // Generate test detection sets
      List<YOLOv8InstantDetection> testDetectionsA = createYoloDetections(3, Instant.now());
      List<CenterPoseInstantDetection> testDetectionsB = createCenterposeDetections(3, Instant.now());

      // add the detection sets to detection manager
      assertDoesNotThrow(() -> detectionManager.addDetections(testDetectionsA));
      assertDoesNotThrow(() -> detectionManager.addDetections(testDetectionsB));

      // check whether detection manager received the sets properly
      List<PersistentDetection> storedDetectionsA = detectionManager.getDetectionsOfType(YOLOv8InstantDetection.class);
      assertEquals(3, storedDetectionsA.size());
      for (PersistentDetection persistentDetection : storedDetectionsA)
      {
         assertNotNull(persistentDetection.getMostRecentDetection());
         assertTrue(testDetectionsA.contains(persistentDetection.getMostRecentDetection()));
      }

      List<PersistentDetection> storedDetectionsB = detectionManager.getDetectionsOfType(CenterPoseInstantDetection.class);
      assertEquals(3, storedDetectionsB.size());
      for (PersistentDetection persistentDetection : storedDetectionsB)
      {
         assertNotNull(persistentDetection.getMostRecentDetection());
         assertTrue(testDetectionsB.contains(persistentDetection.getMostRecentDetection()));
      }
   }

   @Test
   public void testMatchingDetections()
   {
      DetectionManager detectionManager = new DetectionManager(null);

      // Generate the first frame of detections & add to detection manager
      List<YOLOv8InstantDetection> firstFrame = createYoloDetections(3, Instant.now());
      detectionManager.addDetections(firstFrame);

      // Generate second frame of detections & add to detection manager
      List<YOLOv8InstantDetection> secondFrame = createYoloDetections(2, Instant.now());
      detectionManager.addDetections(secondFrame);

      // Ensure detection manager has 3 detections
      List<PersistentDetection> persistentDetections = detectionManager.getDetectionsOfType(YOLOv8InstantDetection.class);
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
   public void testConcurrentDetectionAddition() throws InterruptedException
   {
      testPassed.set(true);
      DetectionManager detectionManager = new DetectionManager(null);

      int numRuns = 1000; // If test is passing when it really shouldn't, try increasing these numbers. Ensure numRuns >= maxDetections
      int maxDetections = 1000;

      // Repeat test numerous times to catch any random race condition
      for (int i = 1; i <= numRuns; ++i)
      {
         int numToGenerate = i % (maxDetections + 1);

         // Two threads attempt to add different classes of detections concurrently. This should not throw exceptions.
         List<YOLOv8InstantDetection> detectionFrameA = createYoloDetections(numToGenerate, Instant.now());
         Thread threadA = new Thread(() ->
         {
            ThreadTools.sleep(random.nextInt(10));
            try
            {
               detectionManager.addDetections(detectionFrameA);
            }
            catch (Exception e)
            {
               e.printStackTrace();
               testPassed.set(false);
            }
         }, "TestThreadA");

         List<CenterPoseInstantDetection> detectionFrameB = createCenterposeDetections(2 * numToGenerate, Instant.now());
         Thread threadB = new Thread (() ->
         {
            ThreadTools.sleep(random.nextInt(10));
            try
            {
               detectionManager.addDetections(detectionFrameB);
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
      List<PersistentDetection> detectionsA = detectionManager.getDetectionsOfType(YOLOv8InstantDetection.class);
      assertEquals(maxDetections, detectionsA.size());

      List<PersistentDetection> detectionsB = detectionManager.getDetectionsOfType(CenterPoseInstantDetection.class);
      assertEquals(2 * maxDetections, detectionsB.size());
   }

   /**
    * Two threads add detections of different classes while the main thread accesses the added detections
    */
   @Test
   public void testConcurrentAdditionAndAccess()
   {
      testPassed.set(true);

      DetectionManager detectionManager = new DetectionManager(null);
      detectionManager.setDetectionHistoryDuration(Duration.ofDays(5));
      detectionManager.setMatchDistanceThreshold(10.0);

      // Create the two addition threads
      final int numRuns = 5000;
      Thread additionThreadA = new Thread(() ->
      {
         for (int i = 0; i < numRuns && testPassed.get(); ++i)
         {
            List<YOLOv8InstantDetection> detectionsA = createYoloDetections(100, Instant.now());
            detectionManager.addDetections(detectionsA);
         }
      }, "AdditionThreadA");

      Thread additionThreadB = new Thread(() ->
      {
         for (int i = 0; i < numRuns && testPassed.get(); ++i)
         {
            List<CenterPoseInstantDetection> detectionsB = createCenterposeDetections(100, Instant.now());
            detectionManager.addDetections(detectionsB);
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

         List<PersistentDetection> detectionsA = detectionManager.getDetectionsOfType(InstantDetection.class);
         List<PersistentDetection> detectionsB = detectionManager.getDetectionsOfType(InstantDetection.class);

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
   public void testDetectionStability()
   {
      DetectionManager detectionManager = new DetectionManager(null);
      detectionManager.setMatchDistanceThreshold(0.5);
      detectionManager.setStabilityAverageConfidence(0.5);
      detectionManager.setStabilityDetectionFrequency(0.9);
      detectionManager.setDetectionHistoryDuration(2.5);

      Instant startTime = Instant.now();

      for (int i = 0; i < 5; ++i)
      {
         List<YOLOv8InstantDetection> testDetectionsA = createYoloDetections(5, startTime.minusSeconds(i));
         detectionManager.addDetections(testDetectionsA);

         List<CenterPoseInstantDetection> testDetectionsB = createCenterposeDetections(5, startTime.minusSeconds(i));
         detectionManager.addDetections(testDetectionsB);
      }

      detectionManager.updateDetections(startTime);

      List<PersistentDetection> persistentDetectionsA = detectionManager.getDetectionsOfType(InstantDetection.class);
      for (PersistentDetection persistentDetection : persistentDetectionsA)
      {
         assertFalse(persistentDetection.isStable());
         assertEquals(3, persistentDetection.getHistorySize());
      }

      List<PersistentDetection> persistentDetectionsB = detectionManager.getDetectionsOfType(InstantDetection.class);
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

   public static List<YOLOv8InstantDetection> createYoloDetections(int numberToGenerate, Instant now)
   {
      List<YOLOv8InstantDetection> testDetections = new ArrayList<>();

      for (int i = 0; i < numberToGenerate; ++i)
      {
         YOLOv8InstantDetection testDetection  = new YOLOv8InstantDetection("detection_" + i, 1.0, new Pose3D(), now, new ArrayList<>());
         testDetections.add(testDetection);
      }

      return testDetections;
   }

   public static List<CenterPoseInstantDetection> createCenterposeDetections(int numberToGenerate, Instant now)
   {
      List<CenterPoseInstantDetection> testDetections = new ArrayList<>();

      for (int i = 0; i < numberToGenerate; ++i)
      {
         CenterPoseInstantDetection testDetection = new CenterPoseInstantDetection("detection_" + i,
                                                                                   "hotdog",
                                                                                   1.0,
                                                                                   new Pose3D(),
                                                                                   now,
                                                                                   new Point3D[3],
                                                                                   new Point2D[4]);
         testDetections.add(testDetection);
      }

      return testDetections;
   }
}
