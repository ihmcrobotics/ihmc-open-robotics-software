package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Pins that the invariant estimator uses the gravity it is <em>given</em> (from the robot process) rather than a
 * hard-coded 9.81. Both tests use lunar gravity, which no constant in this package equals, so a re-hard-coded
 * literal cannot pass them.
 */
public class InvariantEstimatorGravityInjectionTest
{
   private static final double LUNAR_GRAVITY = -1.62;

   /** Free fall integrates the injected |g|, for either input sign (g is always (0, 0, −|g|)). */
   @Test
   public void testFreeFallIntegratesInjectedGravity()
   {
      for (double gravity : new double[] {-9.81, LUNAR_GRAVITY, -LUNAR_GRAVITY})
      {
         double dt = 1.0e-3;
         int steps = 1000;
         InvariantPropagator propagator = new InvariantPropagator(0, 0.0, 0.0, 0.0, gravity);
         InvariantState state = new InvariantState(0);

         Vector3D zero = new Vector3D();
         for (int i = 0; i < steps; i++)
            propagator.predict(state, zero, zero, dt);

         double time = steps * dt;
         Vector3D velocity = new Vector3D();
         Vector3D position = new Vector3D();
         state.getBaseVelocity(velocity);
         state.getBasePosition(position);

         double expectedGz = -Math.abs(gravity);
         assertEquals(expectedGz * time, velocity.getZ(), 1.0e-9, "free fall must use the injected gravity " + gravity);
         assertEquals(0.5 * expectedGz * time * time, position.getZ(), 1.0e-9, "drop must use the injected gravity " + gravity);
      }
   }

   /**
    * The gravity-leveling gate trusts the accelerometer only when ‖a‖ ≈ the injected |g|. At lunar gravity that
    * means 1.62 opens the gate and 9.81 shuts it — the opposite of the hard-coded behaviour this replaced.
    * {@code updateGravityReference} is deliberately not called, so the horizontal term is admitted and only the
    * |g| comparison is under test.
    */
   @Test
   public void testQuasiStaticGateUsesInjectedGravityMagnitude()
   {
      InvariantEKF ekf = new InvariantEKF(0, 1.0e-7, 1.0e-7, 1.0e-12, LUNAR_GRAVITY);
      Vector3D zeroGyro = new Vector3D();

      assertTrue(ekf.isGravityQuasiStatic(new Vector3D(0.0, 0.0, Math.abs(LUNAR_GRAVITY)), zeroGyro, 0.05, 0.05, 0.5),
                 "‖a‖ = injected |g| must be trusted as a gravity reference");
      assertFalse(ekf.isGravityQuasiStatic(new Vector3D(0.0, 0.0, 9.81), zeroGyro, 0.05, 0.05, 0.5),
                  "‖a‖ = 9.81 is 6x the injected |g| and must not be trusted");
   }
}
