package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

/**
 * Property tests for {@link TouchdownReseedLatch}: the touchdown re-seed must fire at most once per
 * sustained-low contact episode. The adversarial signal in
 * {@link #midStrikeDropoutCannotDoubleFire()} is the p ≈ 1 → 0 → 1 double pulse one physical
 * touchdown produced on hardware (log 20260717_112516, joint-torque switch CoP-under-ankle force
 * dropout) — the single-tick re-arm turned it into two re-seeds per strike.
 */
public class TouchdownReseedLatchTest
{
   private static final double TRIGGER = 0.5;
   private static final double REARM = 0.1;
   private static final int DWELL_TICKS = 100; // 100 ms at 1 kHz

   private static int drive(TouchdownReseedLatch latch, double probability, int ticks)
   {
      int fires = 0;
      for (int i = 0; i < ticks; i++)
      {
         if (latch.advance(probability))
            fires++;
      }
      return fires;
   }

   @Test
   public void firesOnceOnCleanTouchdownAndNotAgainWhileHigh()
   {
      TouchdownReseedLatch latch = new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false);
      assertEquals(0, drive(latch, 0.0, DWELL_TICKS)); // clean swing arms, never fires low
      assertTrue(latch.isArmed());
      assertEquals(1, drive(latch, 1.0, 500), "exactly one fire on the rising crossing");
      assertFalse(latch.isArmed());
   }

   @Test
   public void midStrikeDropoutCannotDoubleFire()
   {
      TouchdownReseedLatch latch = new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false);
      drive(latch, 0.0, 500); // swing
      assertEquals(1, drive(latch, 1.0, 90), "touchdown fires once");
      // The mid-strike force dropout: probability collapses for LESS than the re-arm dwell...
      assertEquals(0, drive(latch, 0.0, DWELL_TICKS - 1));
      assertFalse(latch.isArmed(), "sub-dwell dropout must not re-arm");
      // ...then the switch comes back: same physical strike, must NOT re-seed again.
      assertEquals(0, drive(latch, 1.0, 500), "one physical touchdown, one re-seed");
   }

   @Test
   public void sustainedSwingRearmsForTheNextStrike()
   {
      TouchdownReseedLatch latch = new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false);
      drive(latch, 0.0, 500);
      assertEquals(1, drive(latch, 1.0, 200)); // strike N
      assertEquals(0, drive(latch, 0.0, 300)); // genuine swing (≥ dwell) re-arms
      assertTrue(latch.isArmed());
      assertEquals(1, drive(latch, 1.0, 200), "strike N+1 fires again after a real swing");
   }

   @Test
   public void singleTickDipsNeverRearm()
   {
      TouchdownReseedLatch latch = new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false);
      drive(latch, 0.0, 500);
      assertEquals(1, drive(latch, 1.0, 90));
      // Alternate 1-tick dips with high stretches: the dwell counter must reset every time.
      int fires = 0;
      for (int i = 0; i < 1000; i++)
      {
         fires += drive(latch, 0.05, 1);
         fires += drive(latch, 0.9, 5);
      }
      assertEquals(0, fires, "chattering probability must never re-fire the re-seed");
   }

   @Test
   public void midBandProbabilityNeitherArmsNorFires()
   {
      // p between re-arm and trigger (e.g. the sensitive-only 0.5-approach): no dwell accumulates,
      // and an unarmed latch never fires even if p later crosses the trigger.
      TouchdownReseedLatch latch = new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false);
      assertEquals(0, drive(latch, 0.3, 1000));
      assertFalse(latch.isArmed());
      assertEquals(0, drive(latch, 1.0, 100));
   }

   @Test
   public void atMostOneFirePerSustainedLowEpisodeUnderRandomChatter()
   {
      Random random = new Random(1868L);
      TouchdownReseedLatch latch = new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false);
      int consecutiveLow = 0;
      boolean episodeCredit = false; // a completed dwell grants exactly one future fire
      for (int i = 0; i < 200000; i++)
      {
         double p = random.nextDouble();
         boolean fired = latch.advance(p);
         if (p < REARM)
         {
            if (++consecutiveLow >= DWELL_TICKS)
               episodeCredit = true;
         }
         else
         {
            consecutiveLow = 0;
         }
         if (fired)
         {
            assertTrue(episodeCredit, "fired without a preceding sustained-low episode at tick " + i);
            episodeCredit = false;
         }
      }
   }

   @Test
   public void constructorRejectsDegenerateConfigurations()
   {
      assertThrows(IllegalArgumentException.class, () -> new TouchdownReseedLatch(0.5, 0.5, 100, false));
      assertThrows(IllegalArgumentException.class, () -> new TouchdownReseedLatch(0.5, 0.1, 0, false));
   }

   // --- liftoff edge (becameArmedThisTick), used by the contact add/remove mode to marginalize a foot ---

   @Test
   public void becameArmedIsFalseBeforeAnyAdvance()
   {
      assertFalse(new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false).becameArmedThisTick());
   }

   @Test
   public void becameArmedFiresExactlyOnTheDwellCompletionTick()
   {
      TouchdownReseedLatch latch = new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false);
      for (int i = 1; i < DWELL_TICKS; i++) // first DWELL_TICKS-1 low ticks: dwell not yet complete
      {
         latch.advance(0.0);
         assertFalse(latch.becameArmedThisTick(), "liftoff edge too early at tick " + i);
         assertFalse(latch.isArmed());
      }
      latch.advance(0.0); // the DWELL_TICKS-th low tick completes the dwell and re-arms
      assertTrue(latch.becameArmedThisTick(), "liftoff edge on the dwell-completion tick");
      assertTrue(latch.isArmed());

      latch.advance(0.0); // one-shot: stays armed, edge cleared
      assertFalse(latch.becameArmedThisTick(), "liftoff edge must last a single tick");
      assertTrue(latch.isArmed());

      latch.advance(1.0); // high tick clears the edge too
      assertFalse(latch.becameArmedThisTick());
   }

   @Test
   public void touchdownAndLiftoffEdgesAreMutuallyExclusiveOverAFullCycle()
   {
      TouchdownReseedLatch latch = new TouchdownReseedLatch(TRIGGER, REARM, DWELL_TICKS, false);
      for (int cycle = 0; cycle < 3; cycle++)
      {
         boolean sawLiftoff = false;
         for (int i = 0; i < 300; i++) // sustained swing
         {
            boolean fired = latch.advance(0.0);
            assertFalse(fired, "advance must not fire during swing");
            if (latch.becameArmedThisTick())
               sawLiftoff = true;
         }
         assertTrue(sawLiftoff, "a sustained swing must produce exactly one liftoff edge, cycle " + cycle);

         boolean sawTouchdown = false;
         for (int i = 0; i < 200; i++) // touchdown, held high
         {
            boolean fired = latch.advance(1.0);
            assertFalse(fired && latch.becameArmedThisTick(), "touchdown and liftoff edges on the same tick");
            assertFalse(latch.becameArmedThisTick(), "no liftoff edge while high");
            if (fired)
               sawTouchdown = true;
         }
         assertTrue(sawTouchdown, "touchdown must fire once per cycle, cycle " + cycle);
      }
   }
}
