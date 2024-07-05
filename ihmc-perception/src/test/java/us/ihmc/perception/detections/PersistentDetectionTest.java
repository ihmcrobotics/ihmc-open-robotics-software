package us.ihmc.perception.detections;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;

import java.time.Instant;

import static org.junit.jupiter.api.Assertions.*;

public class PersistentDetectionTest
{
   @Test
   public void testGetMostRecentDetection()
   {
      Instant startInstant = Instant.now();

      InstantDetection mostRecentDetection = new InstantDetection("TestDetection", 0.5, new Pose3D(), startInstant);
      PersistentDetection persistentDetection = new PersistentDetection(mostRecentDetection, 1.0, 0.5, 5.0, 1.0);
      for (int i = 1; i < 10; ++i)
      {
         InstantDetection testDetection = new InstantDetection("TestDetection", 1.0, new Pose3D(), startInstant.minusSeconds(i));
         persistentDetection.addDetection(testDetection);
      }
      assertEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());

      persistentDetection.updateHistory(startInstant.plusSeconds(15));
      assertEquals(mostRecentDetection, persistentDetection.getMostRecentDetection());
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
}
