package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.robotics.robotSide.RobotSide;

/** {@link RearFootReleaseLatch}: releases only the unloading, lighter foot in double support, and holds it until it lifts. */
public class RearFootReleaseLatchTest
{
   private static final double RELEASE = 0.5;
   private static final double UNTRUST = 0.25;
   private static final double MIN_DROP = 0.02; // 1.0/s over the 20 ms window

   /** One weight shift: left goes 0.9 -> 0.1 while right goes 0.1 -> 0.9 over n ticks; both trusted throughout. */
   private static RearFootReleaseLatch shift(int n, boolean enabled, int[] releasedAtTick)
   {
      RearFootReleaseLatch latch = new RearFootReleaseLatch();
      for (int k = 0; k < RearFootReleaseLatch.RATE_WINDOW; k++)
         latch.update(enabled, RELEASE, UNTRUST, MIN_DROP, 0.9, 0.1, true, true); // settle the history
      for (int k = 0; k <= n; k++)
      {
         double left = 0.9 - 0.8 * k / n;
         latch.update(enabled, RELEASE, UNTRUST, MIN_DROP, left, 1.0 - left, true, true);
         if (latch.isReleased(RobotSide.LEFT) && releasedAtTick[0] < 0)
            releasedAtTick[0] = k;
         assertFalse(latch.isReleased(RobotSide.RIGHT), "the loading foot must never be released (tick " + k + ")");
      }
      return latch;
   }

   @Test
   public void testTheUnloadingFootIsReleasedOnceBelowTheReleaseLoad()
   {
      int[] at = {-1};
      shift(200, true, at);
      double loadAtRelease = 0.9 - 0.8 * at[0] / 200.0;
      assertTrue(at[0] > 0, "never released");
      assertTrue(loadAtRelease < RELEASE && loadAtRelease > RELEASE - 0.01, "released at load " + loadAtRelease);
   }

   @Test
   public void testTheReleaseHoldsUntilTheFootLiftsEvenAboveTheTrustLoad()
   {
      RearFootReleaseLatch latch = new RearFootReleaseLatch();
      for (int k = 0; k < RearFootReleaseLatch.RATE_WINDOW; k++)
         latch.update(true, RELEASE, UNTRUST, MIN_DROP, 0.9, 0.1, true, true);
      for (int k = 0; k <= 100; k++) // unload the left foot to 0.45 only: it has not lifted
      {
         double left = 0.9 - 0.45 * k / 100.0;
         latch.update(true, RELEASE, UNTRUST, MIN_DROP, left, 1.0 - left, true, true);
      }
      assertTrue(latch.isReleased(RobotSide.LEFT));
      // Load bounces back to 0.4 (above the 0.35 trust load) and stops falling: still released.
      for (int k = 0; k < 50; k++)
         latch.update(true, RELEASE, UNTRUST, MIN_DROP, 0.4, 0.6, false, true);
      assertTrue(latch.isReleased(RobotSide.LEFT));
      // The foot lifts: released clears, and normal hysteresis resumes on the next touchdown.
      latch.update(true, RELEASE, UNTRUST, MIN_DROP, 0.1, 0.9, false, true);
      assertFalse(latch.isReleased(RobotSide.LEFT));
   }

   @Test
   public void testNothingIsReleasedWhenOffOrInSingleSupport()
   {
      int[] at = {-1};
      shift(200, false, at);
      assertTrue(at[0] < 0, "released while disabled");

      RearFootReleaseLatch latch = new RearFootReleaseLatch();
      for (int k = 0; k < 100; k++)
      {
         double left = 0.9 - 0.5 * k / 100.0;
         latch.update(true, RELEASE, UNTRUST, MIN_DROP, left, 1.0 - left, true, false); // right not trusted
         assertFalse(latch.isReleased(RobotSide.LEFT), "released the only trusted foot at tick " + k);
      }
   }

   @Test
   public void testStandingSwayDoesNotReleaseAFoot()
   {
      // Load oscillating 0.4..0.6 between the feet with a 2 s period: the drop over 20 ms is at most 0.006.
      RearFootReleaseLatch latch = new RearFootReleaseLatch();
      for (int k = 0; k < 6000; k++)
      {
         double left = 0.5 + 0.1 * Math.sin(2.0 * Math.PI * k / 2000.0);
         latch.update(true, RELEASE, UNTRUST, MIN_DROP, left, 1.0 - left, true, true);
         assertFalse(latch.isReleased(RobotSide.LEFT), "sway released the left foot at tick " + k);
         assertFalse(latch.isReleased(RobotSide.RIGHT), "sway released the right foot at tick " + k);
      }
   }

   @Test
   public void testAStillLightFootIsNotReleased()
   {
      // Double support at a steady 0.45 / 0.55: below the release load but not unloading.
      RearFootReleaseLatch latch = new RearFootReleaseLatch();
      for (int k = 0; k < 200; k++)
         latch.update(true, RELEASE, UNTRUST, MIN_DROP, 0.45, 0.55, true, true);
      assertFalse(latch.isReleased(RobotSide.LEFT));
      assertFalse(latch.isReleased(RobotSide.RIGHT));
   }
}
