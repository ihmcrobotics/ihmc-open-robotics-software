package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.stateEstimation.invariantEstimator.FootSwitchContactProbabilityProvider.TrustMode;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Property tests for {@link FootSwitchContactProbabilityProvider}'s debounced Schmitt-trigger trust
 * (2026-07-17). The strike signature in {@link #impactBlipIsRejectedAndRealLoadingTrustsOnce()} is
 * taken from hardware log 20260717_112516 t ≈ 2308 (left touchdown): the joint-torque switch's
 * filtered boolean pulses high for ~170 ms with the load percentage above the enter threshold for
 * only ~30 ms (impact spike on a bouncing foot), collapses for ~600 ms (CoP-under-ankle force
 * dropout), then rises for real. The old pipeline consumed that as an up–down–up trust double pulse
 * per physical strike; the properties below pin the debounced behavior:
 * <ol>
 *   <li>an impact blip shorter than the dwell never enters trust, and probability stays strictly
 *       below every 0.5 hard gate in the estimator;</li>
 *   <li>one physical strike produces exactly one trust rise (at genuine load acceptance);</li>
 *   <li>load chatter inside the Schmitt band [stay, enter) cannot toggle a trusted foot;</li>
 *   <li>probability stays in [0, 1] under arbitrary switch/load chatter.</li>
 * </ol>
 */
public class FootSwitchContactProbabilityProviderTest
{
   private static final double DT = 1.0e-3;

   /** Scriptable switch: the test sets the three signals the provider consumes; the rest is inert. */
   private static final class ScriptedFootSwitch implements FootSwitchInterface
   {
      boolean filtered = false;
      boolean sensitive = false;
      double load = 0.0;

      @Override
      public void reset()
      {
      }

      @Override
      public boolean hasFootHitGroundFiltered()
      {
         return filtered;
      }

      @Override
      public boolean hasFootHitGroundSensitive()
      {
         return sensitive;
      }

      @Override
      public double getFootLoadPercentage()
      {
         return load;
      }

      @Override
      public double getCenterOfPressureDistance()
      {
         return Double.NaN;
      }

      @Override
      public FramePoint2DReadOnly getCenterOfPressure()
      {
         return null;
      }

      @Override
      public WrenchReadOnly getMeasuredWrench()
      {
         return null;
      }

      @Override
      public ReferenceFrame getMeasurementFrame()
      {
         return ReferenceFrame.getWorldFrame();
      }
   }

   private final SideDependentList<ScriptedFootSwitch> switches = new SideDependentList<>(new ScriptedFootSwitch(), new ScriptedFootSwitch());
   private final YoRegistry registry = new YoRegistry("test");
   private final FootSwitchContactProbabilityProvider provider = new FootSwitchContactProbabilityProvider(switches, DT, registry);

   private void tick(int ticks, boolean filtered, boolean sensitive, double load)
   {
      ScriptedFootSwitch left = switches.get(RobotSide.LEFT);
      left.filtered = filtered;
      left.sensitive = sensitive;
      left.load = load;
      for (int i = 0; i < ticks; i++)
         provider.update();
   }

   /** Drives the left foot to a settled untrusted state (right foot stays at its init default). */
   private void settleUntrusted()
   {
      tick(500, false, false, 0.0);
      assertFalse(provider.isTrusted(RobotSide.LEFT));
      assertEquals(0.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-6);
   }

   @Test
   public void startsTrustedWithFeetPlanted()
   {
      // Feet are planted when initializeEstimator runs: trust and probability must start at 1 so the
      // filter is anchored from the first tick (no init transient).
      assertTrue(provider.isTrusted(RobotSide.LEFT));
      assertEquals(1.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-12);
      tick(100, true, true, 0.5);
      assertTrue(provider.isTrusted(RobotSide.LEFT));
      assertEquals(1.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-6);
   }

   @Test
   public void impactBlipIsRejectedAndRealLoadingTrustsOnce()
   {
      settleUntrusted();

      int trustRises = 0;
      double maxProbabilityDuringBlip = 0.0;
      boolean previousTrusted = false;

      // Phase 1 — the impact blip (log signature): filtered high 170 ms, load above the enter
      // threshold for only 30 ms around the force spike, sensitive high throughout.
      for (int i = 0; i < 170; i++)
      {
         double load = (i >= 50 && i < 80) ? 0.38 : 0.15;
         tick(1, true, true, load);
         maxProbabilityDuringBlip = Math.max(maxProbabilityDuringBlip, provider.getContactProbability(RobotSide.LEFT));
         if (provider.isTrusted(RobotSide.LEFT) && !previousTrusted)
            trustRises++;
         previousTrusted = provider.isTrusted(RobotSide.LEFT);
      }
      assertEquals(0, trustRises, "impact blip (load above enter threshold < dwell) must not enter trust");
      // Sensitive-only probability approaches 0.5 from below: strictly under every 0.5 hard gate
      // (contact hold, reseed trigger, double-support count).
      assertTrue(maxProbabilityDuringBlip < 0.5, "sensitive-only probability must stay below 0.5, was " + maxProbabilityDuringBlip);

      // Phase 2 — the CoP-under-ankle dropout: 600 ms with force collapsed.
      for (int i = 0; i < 600; i++)
      {
         tick(1, false, false, 0.0);
         if (provider.isTrusted(RobotSide.LEFT) && !previousTrusted)
            trustRises++;
         previousTrusted = provider.isTrusted(RobotSide.LEFT);
      }
      assertEquals(0, trustRises);

      // Phase 3 — genuine load acceptance: sustained filtered + load.
      for (int i = 0; i < 300; i++)
      {
         tick(1, true, true, 0.8);
         if (provider.isTrusted(RobotSide.LEFT) && !previousTrusted)
            trustRises++;
         previousTrusted = provider.isTrusted(RobotSide.LEFT);
      }
      assertEquals(1, trustRises, "one physical strike must produce exactly one trust rise");
      assertTrue(provider.isTrusted(RobotSide.LEFT));
      assertEquals(1.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-6);
   }

   @Test
   public void loadChatterInsideSchmittBandCannotToggleTrust()
   {
      // Trusted at solid load...
      tick(100, true, true, 0.5);
      assertTrue(provider.isTrusted(RobotSide.LEFT));

      // ...then the load oscillates across the ENTER threshold but stays above the STAY threshold:
      // a memoryless threshold would chatter; the Schmitt band must hold trust continuously.
      for (int cycle = 0; cycle < 50; cycle++)
      {
         tick(10, true, true, 0.28);
         assertTrue(provider.isTrusted(RobotSide.LEFT));
         assertEquals(1.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-6);
         tick(10, true, true, 0.45);
         assertTrue(provider.isTrusted(RobotSide.LEFT));
      }

      // Dropping below the stay threshold for the dwell unloads trust.
      tick(100, true, true, 0.20);
      assertFalse(provider.isTrusted(RobotSide.LEFT));
   }

   @Test
   public void subDwellSwitchChatterNeverTrusts()
   {
      settleUntrusted();
      // 20 ms on / 20 ms off, fully loaded when on — each burst is shorter than the 40 ms dwell.
      for (int cycle = 0; cycle < 100; cycle++)
      {
         tick(20, true, true, 1.0);
         assertFalse(provider.isTrusted(RobotSide.LEFT), "sub-dwell burst must not enter trust (cycle " + cycle + ")");
         tick(20, false, false, 0.0);
      }
   }

   @Test
   public void nanLoadDegradesToSwitchOnlyTrust()
   {
      // A switch that cannot compute load (NaN) must not lock the foot out of trust forever:
      // the load tests pass and trust follows the (dwelled) filtered boolean alone.
      settleUntrusted();
      tick(100, true, true, Double.NaN);
      assertTrue(provider.isTrusted(RobotSide.LEFT));
      tick(100, false, false, Double.NaN);
      assertFalse(provider.isTrusted(RobotSide.LEFT));
   }

   @Test
   public void probabilityStaysInUnitIntervalUnderRandomChatter()
   {
      Random random = new Random(24601L);
      for (int i = 0; i < 20000; i++)
      {
         double load = random.nextInt(10) == 0 ? Double.NaN : 1.5 * random.nextDouble();
         tick(1, random.nextBoolean(), random.nextBoolean(), load);
         for (RobotSide side : RobotSide.values)
         {
            double p = provider.getContactProbability(side);
            assertTrue(p >= 0.0 && p <= 1.0, "probability out of [0,1]: " + p);
         }
      }
   }

   @Test
   public void invertedSchmittBandIsRejected()
   {
      assertThrows(IllegalArgumentException.class,
                   () -> new FootSwitchContactProbabilityProvider(switches, DT, new YoRegistry("bad"), 0.8, 0.25, 0.35, 0.04));
   }

   @Test
   public void legacyModeReproducesUndebouncedBehavior()
   {
      // The A/B arm: LEGACY must follow the raw filtered boolean through the EMA — i.e. the impact
      // blip that SCHMITT rejects (see impactBlipIsRejectedAndRealLoadingTrustsOnce) reaches p ≈ 1
      // and collapses again: the pre-2026-07-17 up–down pulse.
      provider.setTrustMode(TrustMode.LEGACY);
      settleUntrusted();

      tick(170, true, true, 0.15); // blip: filtered high, load far below the enter threshold
      assertTrue(provider.getContactProbability(RobotSide.LEFT) > 0.99, "LEGACY must fully trust the raw switch");
      tick(600, false, false, 0.0);
      assertEquals(0.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-6);
      // ... while the Schmitt state machine (still advancing underneath) never entered trust.
      assertFalse(provider.isTrusted(RobotSide.LEFT));
   }

   @Test
   public void noneModePinsFullTrust()
   {
      provider.setTrustMode(TrustMode.NONE);
      Random random = new Random(603L);
      for (int i = 0; i < 5000; i++)
      {
         tick(1, random.nextBoolean(), random.nextBoolean(), 1.5 * random.nextDouble());
         assertEquals(1.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-12, "NONE must pin p = 1 with no lag");
         assertEquals(1.0, provider.getContactProbability(RobotSide.RIGHT), 1.0e-12);
      }
   }

   @Test
   public void liveModeSwitchResumesFromCurrentDebounceState()
   {
      // Untrusted under SCHMITT, pinned to 1 under NONE, then back to SCHMITT: the state machine
      // kept advancing, so p must decay straight back to the (still untrusted) sensitive-only level
      // rather than resuming from stale trust.
      settleUntrusted();
      provider.setTrustMode(TrustMode.NONE);
      tick(50, false, false, 0.0);
      assertEquals(1.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-12);
      provider.setTrustMode(TrustMode.SCHMITT);
      tick(100, false, false, 0.0);
      assertFalse(provider.isTrusted(RobotSide.LEFT));
      assertEquals(0.0, provider.getContactProbability(RobotSide.LEFT), 1.0e-6);
   }
}
