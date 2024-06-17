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

   private static class TestDetectionA extends InstantDetection
   {
      public TestDetectionA(String detectionClass, Instant detectionTime)
      {
         super(detectionClass, 0.0, new Pose3D(), detectionTime);
      }
   }

   private static class TestDetectionB extends InstantDetection
   {
      public TestDetectionB(String detectionClass, Instant detectionTime)
      {
         super(detectionClass, 1.0, new Pose3D(), detectionTime);
      }
   }

   @Test
   public void testAddDetections() throws InvocationTargetException, NoSuchMethodException, InstantiationException, IllegalAccessException
   {
      DetectionManager detectionManager = new DetectionManager();

      // Generate test detection sets
      Set<TestDetectionA> testDetectionsA = generateDetectionFrame(3, TestDetectionA.class);
      Set<TestDetectionB> testDetectionsB = generateDetectionFrame(3, TestDetectionB.class);

      // add the detection sets to detection manager
      assertDoesNotThrow(() -> detectionManager.addDetections(testDetectionsA, TestDetectionA.class));
      assertDoesNotThrow(() -> detectionManager.addDetections(testDetectionsB, TestDetectionB.class));

      // check whether detection manager received the sets properly
      Set<PersistentDetection<TestDetectionA>> storedDetectionsA = detectionManager.getDetectionsOfType(TestDetectionA.class);
      assertEquals(3, storedDetectionsA.size());
      for (PersistentDetection<TestDetectionA> persistentDetection : storedDetectionsA)
      {
         assertNotNull(persistentDetection.getMostRecentDetection());
         assertTrue(testDetectionsA.contains(persistentDetection.getMostRecentDetection()));
      }

      Set<PersistentDetection<TestDetectionB>> storedDetectionsB = detectionManager.getDetectionsOfType(TestDetectionB.class);
      assertEquals(3, storedDetectionsB.size());
      for (PersistentDetection<TestDetectionB> persistentDetection : storedDetectionsB)
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
      Set<TestDetectionA> firstFrame = generateDetectionFrame(3, TestDetectionA.class);
      detectionManager.addDetections(firstFrame, TestDetectionA.class);

      // Generate second frame of detections & add to detection manager
      Set<TestDetectionA> secondFrame = generateDetectionFrame(2, TestDetectionA.class);
      detectionManager.addDetections(secondFrame, TestDetectionA.class);

      // Ensure detection manager has 3 detections
      Set<PersistentDetection<TestDetectionA>> persistentDetections = detectionManager.getDetectionsOfType(TestDetectionA.class);
      assertEquals(3, persistentDetections.size());

      // Ensure detection manager matched only 2 detections, and left one old detection
      int numContained = 0;
      for (PersistentDetection<TestDetectionA> persistentDetection : persistentDetections)
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
         Set<TestDetectionA> detectionFrameA = generateDetectionFrame(numToGenerate, TestDetectionA.class);
         Thread threadA = new Thread(() ->
         {
            ThreadTools.sleep(random.nextInt(10));
            try
            {
               detectionManager.addDetections(detectionFrameA, TestDetectionA.class);
            }
            catch (Exception e)
            {
               e.printStackTrace();
               testPassed.set(false);
            }
         }, "TestThreadA");

         Set<TestDetectionB> detectionFrameB = generateDetectionFrame(2 * numToGenerate, TestDetectionB.class);
         Thread threadB = new Thread (() ->
         {
            ThreadTools.sleep(random.nextInt(10));
            try
            {
               detectionManager.addDetections(detectionFrameB, TestDetectionB.class);
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
      Set<PersistentDetection<TestDetectionA>> detectionsA = detectionManager.getDetectionsOfType(TestDetectionA.class);
      assertEquals(maxDetections, detectionsA.size());

      Set<PersistentDetection<TestDetectionB>> detectionsB = detectionManager.getDetectionsOfType(TestDetectionB.class);
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
            try
            {
               Set<TestDetectionA> detectionsA = generateDetectionFrame(100, TestDetectionA.class);
               detectionManager.addDetections(detectionsA, TestDetectionA.class);
            }
            catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException | InstantiationException e)
            {
               e.printStackTrace();
               testPassed.set(false);
            }
         }
      }, "AdditionThreadA");

      Thread additionThreadB = new Thread(() ->
      {
         for (int i = 0; i < numRuns && testPassed.get(); ++i)
         {
            try
            {
               Set<TestDetectionB> detectionsB = generateDetectionFrame(100, TestDetectionB.class);
               detectionManager.addDetections(detectionsB, TestDetectionB.class);
            }
            catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException | InstantiationException e)
            {
               e.printStackTrace();
               testPassed.set(false);
            }
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

         Set<PersistentDetection<TestDetectionA>> detectionsA = detectionManager.getDetectionsOfType(TestDetectionA.class);
         Set<PersistentDetection<TestDetectionB>> detectionsB = detectionManager.getDetectionsOfType(TestDetectionB.class);

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
            assertEquals(TestDetectionA.class, detection.getInstantDetectionClass());
            assertTrue(detection.getDetectedObjectClass().contains(TestDetectionA.class.getSimpleName()));
         });

         detectionsB.forEach(detection ->
         {
            assertEquals(TestDetectionB.class, detection.getInstantDetectionClass());
            assertTrue(detection.getDetectedObjectClass().contains(TestDetectionB.class.getSimpleName()));
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
      detectionManager.setDefaultStabilityMinHistorySize(3);
      detectionManager.setDefaultHistorySeconds(2.5);

      Instant startTime = Instant.now();

      for (int i = 0; i < 5; ++i)
      {
         Set<TestDetectionA> testDetectionsA = generateDetectionFrame(5, startTime.minusSeconds(i), TestDetectionA.class);
         detectionManager.addDetections(testDetectionsA, TestDetectionA.class);

         Set<TestDetectionB> testDetectionsB = generateDetectionFrame(5, startTime.minusSeconds(i), TestDetectionB.class);
         detectionManager.addDetections(testDetectionsB, TestDetectionB.class);
      }

      detectionManager.updateDetections(startTime);

      Set<PersistentDetection<TestDetectionA>> persistentDetectionsA = detectionManager.getDetectionsOfType(TestDetectionA.class);
      for (PersistentDetection<TestDetectionA> persistentDetection : persistentDetectionsA)
      {
         assertFalse(persistentDetection.isStable());
         assertEquals(3, persistentDetection.getHistorySize());
      }

      Set<PersistentDetection<TestDetectionB>> persistentDetectionsB = detectionManager.getDetectionsOfType(TestDetectionB.class);
      for (PersistentDetection<TestDetectionB> persistentDetection : persistentDetectionsB)
      {
         assertTrue(persistentDetection.isStable());
         assertEquals(3, persistentDetection.getHistorySize());
      }

      Instant future = startTime.plusSeconds(3);
      detectionManager.updateDetections(future);

      for (PersistentDetection<TestDetectionA> persistentDetection : persistentDetectionsA)
      {
         assertFalse(persistentDetection.isStable());
         assertEquals(1, persistentDetection.getHistorySize());
      }

      for (PersistentDetection<TestDetectionB> persistentDetection : persistentDetectionsB)
      {
         assertFalse(persistentDetection.isStable());
         assertEquals(1, persistentDetection.getHistorySize());
      }
   }

   public static <T extends InstantDetection> Set<T> generateDetectionFrame(int numberToGenerate, Class<T> typeToGenerate)
         throws NoSuchMethodException, InvocationTargetException, InstantiationException, IllegalAccessException
   {
      return generateDetectionFrame(numberToGenerate, Instant.now(), typeToGenerate);
   }

   public static <T extends InstantDetection> Set<T> generateDetectionFrame(int numberToGenerate, Instant now, Class<T> typeToGenerate)
         throws NoSuchMethodException, InvocationTargetException, InstantiationException, IllegalAccessException
   {
      Set<T> testDetections = new HashSet<>();

      for (int i = 0; i < numberToGenerate; ++i)
      {
         T testDetection  = typeToGenerate.getDeclaredConstructor(String.class, Instant.class).newInstance(typeToGenerate.getSimpleName() + i, now);
         testDetections.add(testDetection);
      }

      return testDetections;
   }
}
