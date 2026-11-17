package us.ihmc.stateEstimation.invariantEstimator;

import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class ControllerFacingJointFilterTest
{
   private static final double DT = 0.001;
   private static final String HIP = "LEFT_HIP_Y";
   private static final String ANKLE = "LEFT_ANKLE_Y";

   private static double sine(double frequency, int tick)
   {
      return Math.sin(2.0 * Math.PI * frequency * tick * DT);
   }

   /** Peak |output| over the last second of {@code seconds} of a unit sine, the filter's gate set before every tick. */
   private static double steadyStateAmplitude(ControllerFacingJointFilter filter, String joint, double frequency, double seconds)
   {
      int ticks = (int) Math.round(seconds / DT);
      double peak = 0.0;
      for (int tick = 0; tick < ticks; tick++)
      {
         filter.updateGate();
         filter.update(joint, 0.0, sine(frequency, tick));
         if (tick >= ticks - (int) Math.round(1.0 / DT))
            peak = Math.max(peak, Math.abs(filter.getFilteredVelocity()));
      }
      return peak;
   }

   private static ControllerFacingJointFilter newFilter(YoRegistry registry)
   {
      return new ControllerFacingJointFilter(List.of(HIP), DT, registry);
   }

   @Test
   public void testTheDefaultsMatchTheFixedEstimator()
   {
      YoRegistry registry = new YoRegistry("test");
      ControllerFacingJointFilter filter = newFilter(registry);
      assertTrue(((YoBoolean) registry.findVariable("controllerFacingQdNotchEnabled")).getValue());
      assertFalse(((YoBoolean) registry.findVariable("controllerFacingQdFilterInRL")).getValue());
      assertFalse(filter.isInvariantFilterUsingFilteredStates(), "the invariant filter must read the unfiltered states by default");
      assertTrue(ControllerFacingJointFilter.isHipOrKneePitch("RIGHT_KNEE_Y"));
      assertFalse(ControllerFacingJointFilter.isHipOrKneePitch("RIGHT_ANKLE_Y"));
   }

   @Test
   public void testOutsideRLTheAntiDampingBandIsRemovedAndSlowMotionPasses()
   {
      ControllerFacingJointFilter filter = newFilter(new YoRegistry("test"));
      filter.setRLStateActive(false);
      assertTrue(steadyStateAmplitude(filter, HIP, ControllerFacingJointFilter.BAND_STOP_CENTER_HZ, 3.0) < 0.05);
      assertTrue(filter.isActive());
      assertEquals(1.0, steadyStateAmplitude(newFilter(new YoRegistry("test")), HIP, 0.5, 4.0), 0.02, "gait-band motion must pass");
   }

   @Test
   public void testInRLTheControllerGetsTheUnfilteredStatesByDefault()
   {
      ControllerFacingJointFilter filter = newFilter(new YoRegistry("test"));
      filter.setRLStateActive(true);
      int ticks = 2000;
      for (int tick = 0; tick < ticks; tick++)
      {
         filter.updateGate();
         double velocity = sine(ControllerFacingJointFilter.BAND_STOP_CENTER_HZ, tick);
         filter.update(HIP, 0.3, velocity);
         assertEquals(velocity, filter.getFilteredVelocity(), 0.0);
         assertEquals(0.3, filter.getFilteredPosition(), 0.0);
      }
      assertFalse(filter.isActive());
   }

   @Test
   public void testTheFilterCanBeTurnedBackOnInRL()
   {
      YoRegistry registry = new YoRegistry("test");
      ControllerFacingJointFilter filter = newFilter(registry);
      ((YoBoolean) registry.findVariable("controllerFacingQdFilterInRL")).set(true);
      filter.setRLStateActive(true);
      assertTrue(steadyStateAmplitude(filter, HIP, ControllerFacingJointFilter.BAND_STOP_CENTER_HZ, 3.0) < 0.05);
   }

   @Test
   public void testTheMasterSwitchTurnsItOffEverywhere()
   {
      YoRegistry registry = new YoRegistry("test");
      ControllerFacingJointFilter filter = newFilter(registry);
      ((YoBoolean) registry.findVariable("controllerFacingQdNotchEnabled")).set(false);
      filter.setRLStateActive(false);
      assertEquals(1.0, steadyStateAmplitude(filter, HIP, ControllerFacingJointFilter.BAND_STOP_CENTER_HZ, 2.0), 1.0e-3);
   }

   @Test
   public void testOtherJointsPassThrough()
   {
      ControllerFacingJointFilter filter = newFilter(new YoRegistry("test"));
      filter.setRLStateActive(false);
      assertEquals(1.0, steadyStateAmplitude(filter, ANKLE, ControllerFacingJointFilter.BAND_STOP_CENTER_HZ, 2.0), 1.0e-3);
      assertFalse(filter.filtersJoint(ANKLE));
   }

   /** The biquads run while the gate is closed, so leaving RL switches to an already settled output. */
   @Test
   public void testLeavingRLIsBumpless()
   {
      ControllerFacingJointFilter switched = newFilter(new YoRegistry("switched"));
      ControllerFacingJointFilter alwaysOn = newFilter(new YoRegistry("alwaysOn"));
      alwaysOn.setRLStateActive(false);
      double frequency = ControllerFacingJointFilter.BAND_STOP_CENTER_HZ;
      int switchTick = 2000;
      for (int tick = 0; tick < switchTick + 500; tick++)
      {
         switched.setRLStateActive(tick < switchTick);
         switched.updateGate();
         alwaysOn.updateGate();
         double velocity = sine(frequency, tick);
         switched.update(HIP, 0.0, velocity);
         alwaysOn.update(HIP, 0.0, velocity);
         if (tick >= switchTick)
            assertEquals(alwaysOn.getFilteredVelocity(), switched.getFilteredVelocity(), 1.0e-12);
      }
   }
}
