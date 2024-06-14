package us.ihmc.perception.detections;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;

import java.time.Instant;

import static org.junit.jupiter.api.Assertions.*;

public class PersistentDetectionTest
{
   private static class TestDetection extends InstantDetection
   {
      public TestDetection(String detectionClass, double confidence, Instant detectionTime)
      {
         super(detectionClass, confidence, new Pose3D(), detectionTime);
      }

      @Override
      void destroy()
      {

      }
   }

   @Test
   public void testGetMostRecentDetection()
   {
      Instant startInstant = Instant.now();

      TestDetection mostRecentDetection = new TestDetection("TestDetection", 0.5, startInstant);
      PersistentDetection<TestDetection> persistentDetection = new PersistentDetection<>(mostRecentDetection, 1.0);
      for (int i = 1; i < 10; ++i)
      {
         TestDetection testDetection = new TestDetection("TestDetection", 1.0, startInstant.minusSeconds(i));
         persistentDetection.addDetection(testDetection);
      }
      assertEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());

      persistentDetection.updateHistory(startInstant.plusSeconds(15));
      assertEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());
   }

   @Test
   public void testAverageConfidence()
   {
      PersistentDetection<TestDetection> persistentDetection = null;

      Instant startInstant = Instant.now();
      for (int i = 0; i < 10; ++i)
      {
         TestDetection testDetection = new TestDetection("TestDetection", i / 10.0, startInstant.minusSeconds(i));
         if (persistentDetection == null)
            persistentDetection = new PersistentDetection<>(testDetection, 0.5, 10.0);
         else
            persistentDetection.addDetection(testDetection);
      }

      persistentDetection.updateHistory(startInstant);
      assertEquals(0.45, persistentDetection.getAverageConfidence());
   }

   @Test
   public void testIsStableDetection()
   {
      PersistentDetection<TestDetection> persistentDetection = null;

      Instant startInstant = Instant.now();
      for (int i = 0; i < 10; ++i)
      {
         TestDetection testDetection = new TestDetection("TestDetection", i / 10.0, startInstant.minusSeconds(i));
         if (persistentDetection == null)
            persistentDetection = new PersistentDetection<>(testDetection, 0.4, 10.0);
         else
            persistentDetection.addDetection(testDetection);
      }
      assertTrue(persistentDetection.isStable());

      persistentDetection.setHistoryLength(5.0);
      persistentDetection.updateHistory();
      assertFalse(persistentDetection.isStable());

      persistentDetection.setStabilityThreshold(0.19);
      assertTrue(persistentDetection.isStable());
   }

   @Test
   public void testExceptions()
   {
      TestDetection detectionClassA = new TestDetection("ClassA", 1.0, Instant.now());
      TestDetection detectionClassB = new TestDetection("ClassB", 1.0, Instant.now());

      PersistentDetection<TestDetection> persistentDetection = new PersistentDetection<>(detectionClassA, 1.0);

      // Don't allow addition of a different class type
      assertThrows(IllegalArgumentException.class, () -> persistentDetection.addDetection(detectionClassB));

      // Don't allow a negative history length
      assertThrows(IllegalArgumentException.class, () -> persistentDetection.setHistoryLength(-1.0));

      // Don't allow bad arguments in construction
      assertThrows(IllegalArgumentException.class, () -> new PersistentDetection<>(detectionClassA, 1.0, -0.5));
   }
}
