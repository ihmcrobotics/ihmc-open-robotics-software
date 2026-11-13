package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;

import static org.junit.jupiter.api.Assertions.*;

public class PoseHistoryTest
{
   @Test
   public void ringWrapsWithoutRetainingExpiredOrAliasedEntries()
   {
      var history = new PoseHistory();
      var pose = new RigidBodyTransform();
      var result = new RigidBodyTransform();
      long base = 1_000_000_000L;
      for (int i = 0; i < 1000; i++)
      {
         pose.getTranslation().setX(i);
         history.add(base + i * 1_000_000L, pose);
      }
      pose.getTranslation().setX(-100);
      assertFalse(history.getPoseAtTime(base + 699_000_000L, result));
      assertTrue(history.getPoseAtTime(base + 700_500_000L, result));
      assertEquals(700.5, result.getTranslationX(), 1e-12);
      history.add(base + 999_000_000L, pose); // Duplicate cannot change the saved pose.
      assertTrue(history.getPoseAtTime(base + 999_000_000L, result));
      assertEquals(999, result.getTranslationX(), 0);
      history.clear();
      assertFalse(history.getPoseAtTime(base + 999_000_000L, result));
   }

   @Test
   public void interpolatesBetweenEntriesButNeverExtrapolatesOrBridgesLongGaps()
   {
      long wall = 1_793_212_000_000_000_000L, step = 5_000_000L;
      var history = new PoseHistory();
      history.add(wall, pose(0));
      history.add(wall + 2 * step, pose(.02));
      var result = new RigidBodyTransform();
      assertTrue(history.getPoseAtTime(wall + step, result));
      assertEquals(.01, result.getTranslationX(), 1e-12);
      assertFalse(history.getPoseAtTime(wall - 1, null));
      assertFalse(history.getPoseAtTime(wall + 3 * step, null));
      history.add(wall + 100 * step, pose(1));
      assertFalse(history.getPoseAtTime(wall + 50 * step, null));
   }

   private static RigidBodyTransform pose(double x)
   {
      var pose = new RigidBodyTransform();
      pose.getTranslation().setX(x);
      return pose;
   }
}
