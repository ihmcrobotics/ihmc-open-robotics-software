package us.ihmc.stateEstimation.invariant_estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Tests the accelerometer gravity-leveling (tilt) measurement: the roll/pitch observation the contact update
 * does not provide. Verifies the analytic residual/Jacobian on known tilts, that the filter-level update levels
 * pitch/roll and leaves yaw untouched (yaw is unobservable in this filter), and that P stays symmetric PSD.
 */
public class GravityLevelingUpdaterTest
{
   private static final double G = 9.81;
   private static final Vector3D UP = new Vector3D(0.0, 0.0, 1.0);

   /** Estimate upright, robot upright, perfect gravity ⇒ zero residual and zero tilt error. */
   @Test
   public void testUprightGivesZeroResidualAndTilt()
   {
      InvariantState state = new InvariantState(0);
      state.setRotation(new RotationMatrix()); // identity

      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), 2.5e-3, G);
      updater.assemble(state, new Vector3D(0.0, 0.0, G)); // specific force when static & upright

      DMatrixRMaj r = updater.getResidual();
      assertEquals(0.0, r.get(0, 0), 1.0e-12, "residual x");
      assertEquals(0.0, r.get(1, 0), 1.0e-12, "residual y");
      assertEquals(0.0, r.get(2, 0), 1.0e-12, "residual z");
      assertEquals(0.0, updater.getTiltErrorAngle(), 1.0e-12, "tilt angle");
   }

   /**
    * Estimate upright, robot truly PITCHED by θ about +y: the static specific force is g·[−sinθ, 0, cosθ], so
    * the diagnostic must report pitch error −sinθ, ~zero roll, angle |θ|, and residual [−sinθ, 0, cosθ−1].
    */
   @Test
   public void testPitchTiltDiagnosticAndResidual()
   {
      double theta = 0.20;
      InvariantState state = new InvariantState(0);
      state.setRotation(new RotationMatrix()); // estimate = identity

      // True static specific force for a body pitched by +theta about y: a = g · Ry(θ)ᵀ e_z = g·[−sinθ,0,cosθ].
      Vector3D specificForce = new Vector3D(-G * Math.sin(theta), 0.0, G * Math.cos(theta));

      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), 2.5e-3, G);
      updater.assemble(state, specificForce);

      assertEquals(Math.abs(theta), updater.getTiltErrorAngle(), 1.0e-9, "tilt angle = |θ|");
      assertEquals(-Math.sin(theta), updater.getTiltErrorPitch(), 1.0e-9, "pitch error = −sinθ");
      assertEquals(0.0, updater.getTiltErrorRoll(), 1.0e-9, "roll error ≈ 0");

      DMatrixRMaj r = updater.getResidual();
      assertEquals(-Math.sin(theta), r.get(0, 0), 1.0e-9, "residual x = −sinθ");
      assertEquals(0.0, r.get(1, 0), 1.0e-9, "residual y ≈ 0");
      assertEquals(Math.cos(theta) - 1.0, r.get(2, 0), 1.0e-9, "residual z = cosθ − 1");

      // H's δφ block is rank 2 with a null (yaw) direction: column 2 (δφ_z) must be exactly zero.
      DMatrixRMaj h = updater.getMeasurementJacobian();
      for (int row = 0; row < 3; row++)
         assertEquals(0.0, h.get(row, 2), 0.0, "H δφ_z column (yaw) is zero — yaw unobserved");
   }

   /**
    * Filter-level property. The correction is a rotation about a HORIZONTAL WORLD axis (H's world-yaw column is
    * zero), so it levels roll/pitch and never injects world-yaw information — while P stays symmetric PSD.
    *
    * <p>(1) Pure pitch, robot upright ⇒ pitch → 0 with roll and yaw staying exactly 0 (a world-y correction of a
    * pure-pitch estimate stays pure pitch). (2) Already level but yawed ⇒ residual ≈ 0, so the estimate (yaw
    * included) is untouched. Together these isolate the tilt fix from the yaw direction, which is unobservable.</p>
    */
   @Test
   public void testGravityUpdateLevelsTiltAndPreservesYaw()
   {
      Vector3D trueGravity = new Vector3D(0.0, 0.0, G); // robot truly upright ⇒ static specific force = g e_z

      // (1) Pure pitch → levels; roll and yaw stay 0.
      InvariantEKF ekf = new InvariantEKF(0, 1.0e-7, 1.0e-7, 1.0e-12);
      int m = ekf.getState().getTangentSize(); // 9 for N = 0
      RotationMatrix pitched = new RotationMatrix();
      pitched.setYawPitchRoll(0.0, 0.20, 0.0);
      ekf.initialize(pitched, new Vector3D(), new Vector3D(), new Vector3D[0], CommonOps_DDRM.identity(m));

      double initialTilt = tiltAngle(ekf);
      for (int i = 0; i < 200; i++)
      {
         ekf.assembleGravityLeveling(trueGravity);
         ekf.applyGravityLeveling();
         assertTrue(ekf.wasLastUpdateApplied(), "healthy S ⇒ update applied (not gated)");
         assertTrue(Double.isFinite(ekf.getLastConditionProxy()), "cond(S) proxy finite");
         assertSymmetricPSD(ekf.getState().getCovariance());
      }
      assertTrue(tiltAngle(ekf) < 1.0e-3, "pitch leveled: tilt " + initialTilt + " → " + tiltAngle(ekf));
      RotationMatrix finalR = new RotationMatrix();
      ekf.getRotation(finalR);
      assertEquals(0.0, finalR.getYaw(), 1.0e-9, "yaw stays 0 (world-yaw unobserved)");
      assertEquals(0.0, finalR.getRoll(), 1.0e-9, "roll stays 0");

      // (2) Already level but yawed: residual ≈ 0 ⇒ the estimate (yaw included) is untouched.
      InvariantEKF yawedEkf = new InvariantEKF(0, 1.0e-7, 1.0e-7, 1.0e-12);
      RotationMatrix yawed = new RotationMatrix();
      yawed.setYawPitchRoll(0.7, 0.0, 0.0);
      yawedEkf.initialize(yawed, new Vector3D(), new Vector3D(), new Vector3D[0], CommonOps_DDRM.identity(m));
      for (int i = 0; i < 50; i++)
      {
         yawedEkf.assembleGravityLeveling(trueGravity);
         yawedEkf.applyGravityLeveling();
      }
      RotationMatrix yawedFinal = new RotationMatrix();
      yawedEkf.getRotation(yawedFinal);
      assertEquals(0.7, yawedFinal.getYaw(), 1.0e-9, "already-level + yawed: yaw preserved");
      assertTrue(tiltAngle(yawedEkf) < 1.0e-9, "stays level");
   }

   /** Tilt angle = angle between the estimated body up-axis (R̂ᵀ e_z) and world up. */
   private static double tiltAngle(InvariantEKF ekf)
   {
      RotationMatrix r = new RotationMatrix();
      ekf.getRotation(r);
      Vector3D bodyUp = new Vector3D();
      r.inverseTransform(UP, bodyUp); // R̂ᵀ e_z
      return Math.acos(Math.min(1.0, Math.max(-1.0, bodyUp.dot(UP))));
   }

   private static void assertSymmetricPSD(DMatrixRMaj p)
   {
      int n = p.getNumRows();
      for (int i = 0; i < n; i++)
         for (int j = i + 1; j < n; j++)
            assertEquals(p.get(i, j), p.get(j, i), 1.0e-9, "P symmetric at (" + i + "," + j + ")");
      // Cholesky succeeds ⇒ PD (a small jitter guards the exactly-singular directions).
      DMatrixRMaj jittered = p.copy();
      for (int i = 0; i < n; i++)
         jittered.add(i, i, 1.0e-12);
      org.ejml.interfaces.decomposition.CholeskyDecomposition_F64<DMatrixRMaj> chol =
            new org.ejml.dense.row.decomposition.chol.CholeskyDecompositionInner_DDRM(true);
      assertTrue(chol.decompose(jittered), "P is PSD (Cholesky of P + εI succeeds)");
   }
}
