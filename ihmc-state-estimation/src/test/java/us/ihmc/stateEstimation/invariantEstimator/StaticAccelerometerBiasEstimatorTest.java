package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * The held start-up accelerometer bias: estimated along gravity from a window of standing still, zero until
 * then, restarted by motion, and re-estimable on request.
 */
public class StaticAccelerometerBiasEstimatorTest
{
   private static final double DT = 0.001;
   private static final double G = 9.81;

   /** The 2026-07-17 pelvis IMU at rest: tilted, reading 9.561 m/s^2. */
   private static Vector3D restingReading()
   {
      Vector3D a = new Vector3D(0.0968, 0.2129, 9.558);
      a.scale(9.561 / a.norm());
      return a;
   }

   @Test
   public void testStandingStillHoldsTheBiasAlongGravityAfterTheWindow()
   {
      StaticAccelerometerBiasEstimator estimator = new StaticAccelerometerBiasEstimator(-G, 3.0, 0.05, new YoRegistry("test"));
      Vector3D a = restingReading();
      for (int t = 0; t < 2999; t++)
      {
         estimator.update(a, 0.001, DT);
         assertEquals(0.0, estimator.getBias().norm(), 0.0, "the bias must stay zero until the window completes");
      }
      estimator.update(a, 0.001, DT);
      assertEquals(StaticAccelerometerBiasEstimator.State.HELD, estimator.getState());
      assertEquals(0.249, estimator.getBias().norm(), 1.0e-9);
      Vector3D corrected = new Vector3D(a);
      corrected.sub(estimator.getBias());
      assertEquals(G, corrected.norm(), 1.0e-9, "the corrected reading at rest is exactly g");
      Vector3D cross = new Vector3D();
      cross.cross(estimator.getBias(), a);
      assertEquals(0.0, cross.norm(), 1.0e-9, "the bias lies along the measured gravity direction");

      Vector3D held = new Vector3D(estimator.getBias());
      Vector3D walking = new Vector3D(3.0, -1.0, 12.0);
      for (int t = 0; t < 5000; t++)
         estimator.update(walking, 2.0, DT);
      assertEquals(held, new Vector3D(estimator.getBias()), "once held, later motion must not move the estimate");
   }

   @Test
   public void testMotionInsideTheWindowRestartsIt()
   {
      YoRegistry registry = new YoRegistry("test");
      StaticAccelerometerBiasEstimator estimator = new StaticAccelerometerBiasEstimator(G, 1.0, 0.05, registry);
      Vector3D a = restingReading();
      for (int t = 0; t < 900; t++)
         estimator.update(a, 0.0, DT);
      estimator.update(a, 0.5, DT); // a bump
      for (int t = 0; t < 999; t++)
         estimator.update(a, 0.0, DT);
      assertEquals(StaticAccelerometerBiasEstimator.State.COLLECTING, estimator.getState(), "the 900 ticks before the bump must not count");
      estimator.update(a, 0.0, DT);
      assertEquals(StaticAccelerometerBiasEstimator.State.HELD, estimator.getState());
      assertEquals(1, ((us.ihmc.yoVariables.variable.YoInteger) registry.findVariable("staticAccelBiasRestarts")).getValue());
   }

   @Test
   public void testANonFiniteSampleRestartsTheWindowInsteadOfPoisoningIt()
   {
      StaticAccelerometerBiasEstimator estimator = new StaticAccelerometerBiasEstimator(G, 0.5, 0.05, null);
      Vector3D a = restingReading();
      for (int t = 0; t < 300; t++)
         estimator.update(a, 0.0, DT);
      estimator.update(new Vector3D(Double.NaN, 0.0, 9.8), 0.0, DT);
      for (int t = 0; t < 500; t++)
         estimator.update(a, 0.0, DT);
      assertEquals(StaticAccelerometerBiasEstimator.State.HELD, estimator.getState());
      assertTrue(Double.isFinite(estimator.getBias().norm()));
      assertEquals(0.249, estimator.getBias().norm(), 1.0e-9);
   }

   @Test
   public void testReestimateDiscardsTheHeldValueAndCollectsAgain()
   {
      YoRegistry registry = new YoRegistry("test");
      StaticAccelerometerBiasEstimator estimator = new StaticAccelerometerBiasEstimator(G, 0.2, 0.05, registry);
      for (int t = 0; t < 200; t++)
         estimator.update(restingReading(), 0.0, DT);
      assertEquals(StaticAccelerometerBiasEstimator.State.HELD, estimator.getState());

      ((YoBoolean) registry.findVariable("staticAccelBiasReestimate")).set(true);
      Vector3D other = new Vector3D(0.0, 0.0, 9.70);
      estimator.update(other, 0.0, DT);
      assertEquals(StaticAccelerometerBiasEstimator.State.COLLECTING, estimator.getState());
      assertEquals(0.0, estimator.getBias().norm(), 0.0);
      for (int t = 0; t < 199; t++)
         estimator.update(other, 0.0, DT);
      assertEquals(StaticAccelerometerBiasEstimator.State.HELD, estimator.getState());
      assertEquals(-0.11, estimator.getBias().getZ(), 1.0e-9);
   }
}
