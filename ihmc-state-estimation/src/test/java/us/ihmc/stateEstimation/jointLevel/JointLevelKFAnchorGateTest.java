package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The stance anchor assumes the trusted foot does not turn. Its sensor-only residual |omega_base + J_leg qd| is
 * that foot's angular rate; a rolling foot (heel strike, toe-off) must be dropped rather than read as base gyro
 * bias.
 */
public class JointLevelKFAnchorGateTest
{
   @Test
   public void testRollingFootIsDroppedAndPlantedFootKept()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.twoPairs(4242L, 10, 1, 5, 9);
      YoInteger active = (YoInteger) f.registry.findVariable("jointKFActiveAnchorCount");
      YoInteger gated = (YoInteger) f.registry.findVariable("jointKFAnchorGatedCount");
      YoDouble gate = (YoDouble) f.registry.findVariable("jointKFParam_anchorResidualGate");
      assertEquals(JointKFParameters.ANCHOR_RESIDUAL_GATE, gate.getValue());

      // Planted: every gyro and joint rate zero, plus a 1 mrad/s base bias -> residual 1e-3, kept.
      for (JointLevelKFTestFixture.TestIMU imu : f.imus)
         imu.setAngularVelocity(0.0, 0.0, 0.0);
      f.imus.get(0).setAngularVelocity(0.0, 1.0e-3, 0.0);
      f.filter.setTrustedFeetForTest(new ArrayList<>(f.feet));
      f.filter.buildStackedMeasurementForTest();
      assertEquals(f.feet.size(), active.getValue());
      assertEquals(0, gated.getValue());

      // The base turns at 0.3 rad/s with no joint motion to explain it: the foot is turning with it.
      f.imus.get(0).setAngularVelocity(0.0, 0.3, 0.0);
      f.filter.buildStackedMeasurementForTest();
      assertEquals(0, active.getValue());
      assertTrue(gated.getValue() >= 1);

      // Disabled gate: the old behaviour, anchor kept whatever the residual.
      gate.set(0.0);
      f.filter.buildStackedMeasurementForTest();
      assertEquals(f.feet.size(), active.getValue());
   }
}
