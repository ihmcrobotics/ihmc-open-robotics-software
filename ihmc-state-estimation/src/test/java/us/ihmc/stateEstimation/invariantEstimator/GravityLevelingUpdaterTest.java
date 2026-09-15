package us.ihmc.stateEstimation.invariantEstimator;

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
      InvariantEKF ekf = new InvariantEKF(0, 1.0e-7, 1.0e-7, 1.0e-12, G);
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
      InvariantEKF yawedEkf = new InvariantEKF(0, 1.0e-7, 1.0e-7, 1.0e-12, G);
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

   private static final double ROLL_VAR = 2.5e-3;   // (2.9°)²
   private static final double PITCH_VAR = 1.9e-1;  // (25°)²
   private static final double DT = 1.0e-3;         // estimator tick, for driving the gate's gravity reference

   /**
    * Anisotropic R structure. At an upright estimate û_p = R̂ᵀe_x = e_x, so R must be diag(σ_pitch², σ_roll²,
    * σ_roll²): the pitch-informing residual direction (x) carries the loose σ_pitch², roll (y) and the gravity
    * null (z) carry the tight σ_roll². Off-diagonals zero; symmetric.
    */
   @Test
   public void testAnisotropicMeasurementCovarianceStructure()
   {
      InvariantState state = new InvariantState(0);
      state.setRotation(new RotationMatrix()); // identity

      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), ROLL_VAR, PITCH_VAR, G);
      updater.assemble(state, new Vector3D(0.0, 0.0, G));

      DMatrixRMaj r = updater.getMeasurementCovariance();
      assertEquals(PITCH_VAR, r.get(0, 0), 1.0e-12, "σ_pitch² on the pitch-informing (x) direction");
      assertEquals(ROLL_VAR, r.get(1, 1), 1.0e-12, "σ_roll² on the roll-informing (y) direction");
      assertEquals(ROLL_VAR, r.get(2, 2), 1.0e-12, "σ_roll² on the gravity-null (z) direction");
      for (int i = 0; i < 3; i++)
         for (int j = i + 1; j < 3; j++)
         {
            assertEquals(0.0, r.get(i, j), 1.0e-12, "R off-diagonal (" + i + "," + j + ") zero at identity");
            assertEquals(r.get(i, j), r.get(j, i), 1.0e-15, "R symmetric");
         }
   }

   /**
    * The core anisotropy property: at a realistic (converged) covariance, ONE gravity update corrects far less
    * PITCH than ROLL for the same tilt magnitude. The per-tick Kalman gain is P/(P+σ²), so the pitch:roll
    * correction ratio is ≈ (P+σ_roll²)/(P+σ_pitch²) — with σ_pitch² ≫ σ_roll² the accelerometer barely nudges
    * pitch each tick (so fore-aft accel that slips the gate can't yank the base forward in a few ticks) while
    * still fully authoritative on roll (the JointKF's roll drift). Steady-state pitch is instead set by the gyro
    * propagation, and sustained fore-aft contamination is rejected by the quasi-static gate, not by R.
    */
   @Test
   public void testPitchCorrectionAuthorityBelowRoll()
   {
      double theta = 0.10;              // small tilt to level
      double p0 = 1.0e-2;               // a converged-filter covariance (NOT the initial P=1)

      double rollReduction = theta - oneStepTiltAfterCorrection(0.0, 0.0, theta, p0); // rolled by θ → level
      double pitchReduction = theta - oneStepTiltAfterCorrection(0.0, theta, 0.0, p0); // pitched by θ → level

      assertTrue(rollReduction > 5.0 * pitchReduction,
                 "roll correction/tick (" + rollReduction + ") must dominate pitch (" + pitchReduction + ")");
      assertTrue(pitchReduction > 0.0, "pitch still corrects (finite σ_pitch, not frozen)");
   }

   /** Initializes upright-except-(yaw,pitch,roll) at covariance p0·I, applies ONE gravity update from true gravity, returns the residual tilt angle. */
   private static double oneStepTiltAfterCorrection(double yaw, double pitch, double roll, double p0)
   {
      InvariantEKF ekf = new InvariantEKF(0, 1.0e-7, 1.0e-7, 1.0e-12, G); // default anisotropic (roll tight, pitch loose)
      int m = ekf.getState().getTangentSize();
      RotationMatrix r0 = new RotationMatrix();
      r0.setYawPitchRoll(yaw, pitch, roll);
      DMatrixRMaj p = CommonOps_DDRM.identity(m);
      CommonOps_DDRM.scale(p0, p);
      ekf.initialize(r0, new Vector3D(), new Vector3D(), new Vector3D[0], p);

      ekf.assembleGravityLeveling(new Vector3D(0.0, 0.0, G)); // robot truly upright ⇒ static specific force = g e_z
      ekf.applyGravityLeveling();
      assertSymmetricPSD(ekf.getState().getCovariance());
      return tiltAngle(ekf);
   }

   /** Roll is fully trusted, so a pure roll tilt still levels — the JointKF's roll fix survives the anisotropy. */
   @Test
   public void testRollStillLevelsUnderAnisotropy()
   {
      Vector3D trueGravity = new Vector3D(0.0, 0.0, G);
      InvariantEKF ekf = new InvariantEKF(0, 1.0e-7, 1.0e-7, 1.0e-12, G); // default anisotropic
      int m = ekf.getState().getTangentSize();
      RotationMatrix rolled = new RotationMatrix();
      rolled.setYawPitchRoll(0.0, 0.0, 0.20);
      ekf.initialize(rolled, new Vector3D(), new Vector3D(), new Vector3D[0], CommonOps_DDRM.identity(m));

      for (int i = 0; i < 200; i++)
      {
         ekf.assembleGravityLeveling(trueGravity);
         ekf.applyGravityLeveling();
      }
      assertTrue(tiltAngle(ekf) < 1.0e-3, "roll leveled under anisotropic noise: tilt → " + tiltAngle(ekf));
   }

   /**
    * The double-support pitch gate: {@code setPitchObservable(false)} must freeze the pitch correction (σ_pitch²
    * → disabled) while STILL leveling roll — so single-support keeps the roll fix without the pitch lean.
    */
   @Test
   public void testPitchGateFreezesPitchButNotRoll()
   {
      InvariantState state = new InvariantState(0);
      state.setRotation(new RotationMatrix());
      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), ROLL_VAR, PITCH_VAR, G);
      updater.setPitchObservable(false);
      updater.assemble(state, new Vector3D(0.0, 0.0, G));

      DMatrixRMaj r = updater.getMeasurementCovariance();
      assertTrue(r.get(0, 0) > 1.0e3, "pitch direction variance jumps to the disabled value (" + r.get(0, 0) + ")");
      assertEquals(ROLL_VAR, r.get(1, 1), 1.0e-12, "roll direction stays trusted");
   }

   /**
    * Horizontal-accel gate: a fore-aft specific force whose NORM still sits within ±5% of |g| (so the norm gate
    * passes) is rejected by the horizontal component, while pure vertical gravity passes.
    */
   @Test
   public void testHorizontalAccelGateRejectsForeAftButPassesGravity()
   {
      InvariantState state = new InvariantState(0);
      state.setRotation(new RotationMatrix());
      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), ROLL_VAR, PITCH_VAR, G);
      Vector3D zeroOmega = new Vector3D();
      Vector3D gravity = new Vector3D(0.0, 0.0, G);

      // The gate's gravity reference is now driven from the sensors (as production does every tick), not read
      // off the estimate. Settle it on pure gravity first.
      settleGravityReference(updater, gravity);

      updater.assemble(state, gravity);
      assertTrue(updater.isQuasiStatic(gravity, zeroOmega, 0.05, 0.15, 0.5), "pure gravity passes the gate");

      // 3 m/s² fore-aft with a slightly reduced vertical: ‖a‖ ≈ 9.36 m/s², within ±5% of g (norm gate passes)…
      Vector3D foreAft = new Vector3D(3.0, 0.0, Math.sqrt(G * G - 3.0 * 3.0));
      updater.updateGravityReference(foreAft, zeroOmega, DT); // one tick: the slow reference barely moves
      updater.assemble(state, foreAft);
      assertTrue(Math.abs(foreAft.norm() - G) <= 0.05 * G, "sanity: norm gate alone would pass");
      assertTrue(!updater.isQuasiStatic(foreAft, zeroOmega, 0.05, 0.15, 0.5),
                 "…but the horizontal-accel gate rejects the fore-aft component");
   }

   /**
    * THE REGRESSION TEST FOR THE PITCH DRIFT (FINDINGS.md §F.3).
    *
    * <p>The robot is genuinely static: the accelerometer reads pure gravity and the gyro reads zero. The gate
    * must therefore say "quasi-static" NO MATTER WHAT the filter's attitude estimate is — the robot's motion is
    * a fact about the world, not about our belief. The old gate resolved the horizontal specific force against
    * {@code ĝ = R̂ᵀe_z}, which for a static robot evaluates to {@code g·sin θ} with θ the tilt error itself, so
    * it switched the corrector OFF for θ > arcsin(0.5/9.81) = 2.92° and could never re-close. Hardware log
    * 20260712_163634 latched at t=597 s and never leveled again for the remaining 247 s.
    */
   @Test
   public void testQuasiStaticGateIsIndependentOfEstimatorAttitude()
   {
      GravityLevelingUpdater updater = new GravityLevelingUpdater(new InvariantState(0).getTangentSize(), ROLL_VAR, PITCH_VAR, G);
      Vector3D zeroOmega = new Vector3D();
      Vector3D trueGravity = new Vector3D(0.0, 0.0, G); // robot upright and STATIC: this is ground truth

      settleGravityReference(updater, trueGravity);

      // Sweep the ESTIMATE's tilt error from 0 to 15°, well past the old 2.92° lockout. The measurement is
      // unchanged throughout — only our (wrong) belief moves.
      for (int degrees = 0; degrees <= 15; degrees++)
      {
         InvariantState state = new InvariantState(0);
         RotationMatrix wrongEstimate = new RotationMatrix();
         wrongEstimate.setToPitchOrientation(Math.toRadians(degrees));
         state.setRotation(wrongEstimate);
         updater.assemble(state, trueGravity);

         assertTrue(updater.isQuasiStatic(trueGravity, zeroOmega, 0.05, 0.15, 0.5),
                    "the robot is static, so the gate MUST stay open at a tilt error of " + degrees
                    + "° — a corrector whose gate depends on its own error is unstable by construction");
      }
   }

   /**
    * The gate must not be closable by a diverging upstream gyro bias either — {@code lowRotation} takes the RAW
    * gyro. (Second latent latch: on hardware the bias-corrected ‖ω‖ reached 0.074 rad/s against a 0.15 gate
    * while the robot stood still.)
    */
   @Test
   public void testRotationGateUsesRawGyroNotBiasCorrupted()
   {
      InvariantState state = new InvariantState(0);
      state.setRotation(new RotationMatrix());
      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), ROLL_VAR, PITCH_VAR, G);
      Vector3D gravity = new Vector3D(0.0, 0.0, G);
      settleGravityReference(updater, gravity);
      updater.assemble(state, gravity);

      assertTrue(updater.isQuasiStatic(gravity, new Vector3D(0.0, 0.0, 0.0), 0.05, 0.15, 0.5),
                 "a truly still robot (raw gyro ~ 0) passes the rotation gate");
      assertTrue(!updater.isQuasiStatic(gravity, new Vector3D(0.0, 0.3, 0.0), 0.05, 0.15, 0.5),
                 "a genuinely rotating robot is still rejected");
   }

   /**
    * The anisotropic R must distrust BODY pitch. At non-zero yaw the old {@code û_p = R̂ᵀe_x} points at the
    * residual direction of a WORLD-Y rotation instead, so the 76× distrust lands on the wrong axis (the hardware
    * log's yaw was −0.32 rad ≈ 18° off). See jointkf_bias_observability.md §4.
    */
   @Test
   public void testPitchDistrustAxisIsBodyYAtNonZeroYaw()
   {
      InvariantState state = new InvariantState(0);
      RotationMatrix yawed = new RotationMatrix();
      yawed.setToYawOrientation(Math.toRadians(90.0));
      state.setRotation(yawed);

      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), ROLL_VAR, PITCH_VAR, G);
      Vector3D gravityBody = new Vector3D();
      yawed.inverseTransform(new Vector3D(0.0, 0.0, G), gravityBody); // what a static, yawed robot measures
      updater.assemble(state, gravityBody);

      DMatrixRMaj R = updater.getMeasurementCovariance();
      Vector3D gHat = new Vector3D(gravityBody);
      gHat.normalize();

      // Residual of a pure BODY-pitch error is e_y × ĝ_body; of a pure BODY-roll error, e_x × ĝ_body.
      Vector3D pitchResidualDir = new Vector3D();
      pitchResidualDir.cross(new Vector3D(0.0, 1.0, 0.0), gHat);
      pitchResidualDir.normalize();
      Vector3D rollResidualDir = new Vector3D();
      rollResidualDir.cross(new Vector3D(1.0, 0.0, 0.0), gHat);
      rollResidualDir.normalize();

      assertEquals(PITCH_VAR, quadraticForm(R, pitchResidualDir), 1.0e-9,
                   "the body-pitch residual direction must carry sigma_pitch^2, even at 90 deg of yaw");
      assertEquals(ROLL_VAR, quadraticForm(R, rollResidualDir), 1.0e-9,
                   "the body-roll residual direction must still carry sigma_roll^2");
   }

   /** uᵀ R u. */
   private static double quadraticForm(DMatrixRMaj R, Vector3D u)
   {
      double[] v = {u.getX(), u.getY(), u.getZ()};
      double sum = 0.0;
      for (int r = 0; r < 3; r++)
         for (int c = 0; c < 3; c++)
            sum += v[r] * R.get(r, c) * v[c];
      return sum;
   }

   /** Drives the sensor-only gravity reference to steady state on a constant specific force, as production does. */
   private static void settleGravityReference(GravityLevelingUpdater updater, Vector3D specificForce)
   {
      Vector3D zeroOmega = new Vector3D();
      for (int i = 0; i < 3000; i++)
         updater.updateGravityReference(specificForce, zeroOmega, DT);
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

   // ==================================================================================================
   // Roll-sway property tests (Alex002 hardware log 20260712_185914).
   //
   // A balancing biped violates the quasi-static assumption in the worst possible way: the lateral CoM
   // acceleration ÿ that the balance controller produces IN RESPONSE TO the roll this update publishes shows
   // up in the accelerometer as an apparent tilt ÿ/g of the SAME sign — positive feedback through the
   // controller. On hardware the roll estimate oscillated at ω_b = 3.08 rad/s (0.49 Hz) with 6.5× the true
   // tilt amplitude, growing until the robot fell.
   //
   // The fix is a bandwidth argument, and it has TWO halves that must BOTH hold. Testing only the first would
   // pass for a naive low-pass filter, which would destroy the update. So:
   //   (1) the ÿ/g artifact must be REJECTED at ω_b, and
   //   (2) TRUE tilt must still pass at UNITY GAIN at that same ω_b (the gyro carries it).
   // Together these are the defining property of the complementary gravity reference.
   // ==================================================================================================

   /** Lateral balance mode measured on hardware: 0.49 Hz. */
   private static final double BALANCE_OMEGA = 2.0 * Math.PI * 0.49;
   /** Theoretical artifact rejection of the τ-low-pass at ω_b: 1/√(1+(ω_b τ)²). τ = 5 s ⇒ ≈ 0.065 (15×). */
   private static final double PREDICTED_ARTIFACT_GAIN = 1.0 / Math.sqrt(1.0 + Math.pow(BALANCE_OMEGA * 5.0, 2));

   /**
    * (1) ARTIFACT REJECTION. Body perfectly upright and NOT rotating (ω = 0), but subjected to a sinusoidal
    * lateral specific force at the balance frequency — exactly the swaying-robot case. The true tilt is zero, so
    * every bit of residual the updater produces is a lie. Assert the residual the filter is fed is attenuated by
    * at least 10× relative to the raw accelerometer's apparent tilt (ÿ/g), i.e. close to the predicted 15×.
    *
    * <p>Before the fix this test would see gain ≈ 1.0 (the raw f̂ was the measurement) — the residual WAS the
    * artifact, and the filter believed all of it.
    */
   @Test
   public void testLateralAccelArtifactIsRejectedAtTheBalanceFrequency()
   {
      InvariantState state = new InvariantState(0);
      state.setRotation(new RotationMatrix()); // upright, and it never rotates: TRUE TILT IS ZERO THROUGHOUT

      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), ROLL_VAR, PITCH_VAR, G);
      Vector3D zeroOmega = new Vector3D();

      double lateralAccelAmplitude = 0.21;                       // m/s², the amplitude measured on hardware
      double rawArtifactAmplitude = lateralAccelAmplitude / G;   // what the RAW accelerometer calls "roll tilt"

      settleGravityReference(updater, new Vector3D(0.0, 0.0, G));

      double maxResidualY = 0.0;
      // Run several full periods past the reference's transient, then measure the residual amplitude.
      int ticks = (int) (12.0 / DT);
      int measureAfter = (int) (8.0 / DT);
      for (int i = 0; i < ticks; i++)
      {
         double t = i * DT;
         // Specific force of an upright, laterally accelerating body: a = Rᵀ(a_world − g) with R = I.
         Vector3D specificForce = new Vector3D(0.0, lateralAccelAmplitude * Math.sin(BALANCE_OMEGA * t), G);
         updater.updateGravityReference(specificForce, zeroOmega, DT);
         updater.assemble(state, specificForce);
         if (i > measureAfter)
            maxResidualY = Math.max(maxResidualY, Math.abs(updater.getResidual().get(1, 0)));
      }

      double achievedGain = maxResidualY / rawArtifactAmplitude;
      assertTrue(achievedGain < 0.1,
                 "the lateral-accel artifact must be rejected by >10x at the balance mode, but the update was fed "
                 + achievedGain + " of it (predicted " + PREDICTED_ARTIFACT_GAIN + "). A gain near 1.0 means the "
                 + "update is again correcting against the raw specific force -> positive feedback -> roll sway.");
      // And it should actually match the first-order theory, not merely be small for some accidental reason.
      assertEquals(PREDICTED_ARTIFACT_GAIN, achievedGain, 0.03, "artifact gain should match 1/sqrt(1+(w*tau)^2)");
   }

   /**
    * (2) TRUE-TILT PASSTHROUGH — the property that forbids "just low-pass the accelerometer". The body genuinely
    * rolls sinusoidally at the SAME balance frequency, with a gyro consistent with that motion and an
    * accelerometer that reads pure (rotated) gravity. The gravity reference must track the true body-frame
    * gravity direction with UNITY gain — the gyro supplies exactly the content the accelerometer low-pass
    * removes. A naive low-pass of f̂ would lag/attenuate here and fail.
    */
   @Test
   public void testTrueTiltStillPassesAtUnityGainAtTheBalanceFrequency()
   {
      GravityLevelingUpdater updater = new GravityLevelingUpdater(new InvariantState(0).getTangentSize(), ROLL_VAR, PITCH_VAR, G);

      double rollAmplitude = 0.03; // rad, ~1.7°

      settleGravityReference(updater, new Vector3D(0.0, 0.0, G));

      double maxTrackingError = 0.0;
      int ticks = (int) (12.0 / DT);
      int measureAfter = (int) (8.0 / DT);
      for (int i = 0; i < ticks; i++)
      {
         double t = i * DT;
         double roll = rollAmplitude * Math.sin(BALANCE_OMEGA * t);
         double rollRate = rollAmplitude * BALANCE_OMEGA * Math.cos(BALANCE_OMEGA * t);

         RotationMatrix trueRotation = new RotationMatrix();
         trueRotation.setYawPitchRoll(0.0, 0.0, roll);

         // A truly tilting, non-translating body: a = Rᵀ(0 − g) = Rᵀ (0,0,g). Gyro sees the real roll rate.
         Vector3D specificForce = new Vector3D();
         trueRotation.inverseTransform(new Vector3D(0.0, 0.0, G), specificForce);
         Vector3D omega = new Vector3D(rollRate, 0.0, 0.0);

         updater.updateGravityReference(specificForce, omega, DT);

         if (i > measureAfter)
         {
            // Truth: the world up-axis expressed in body frame.
            Vector3D trueGravityBody = new Vector3D();
            trueRotation.inverseTransform(UP, trueGravityBody);
            Vector3D error = new Vector3D(updater.getGravityReference());
            error.sub(trueGravityBody);
            maxTrackingError = Math.max(maxTrackingError, error.norm());
         }
      }

      // Unity gain: the reference must follow the real tilt to well within a tenth of its amplitude.
      assertTrue(maxTrackingError < 0.1 * rollAmplitude,
                 "the gravity reference must track TRUE tilt at unity gain (gyro-carried) at the balance mode; "
                 + "tracking error was " + maxTrackingError + " rad against a " + rollAmplitude
                 + " rad true roll. If this fails, the fix has degenerated into a naive low-pass and the update "
                 + "can no longer see real tilt.");
   }

   /**
    * (3) DC AUTHORITY PRESERVED. The whole point of this update is to hold roll/pitch against a residual gyro
    * bias, which is a DC problem. Low-passing the measurement must not cost any DC gain: a STATIC tilt must
    * still produce the full residual, so the filter still levels a standing lean. (This is what a σ_roll²
    * detune — the obvious alternative fix — would have thrown away.)
    */
   @Test
   public void testStaticTiltStillProducesFullResidual()
   {
      double roll = 0.05; // rad, a 2.9° standing lean

      InvariantState state = new InvariantState(0);
      state.setRotation(new RotationMatrix()); // the ESTIMATE thinks it is upright...

      GravityLevelingUpdater updater = new GravityLevelingUpdater(state.getTangentSize(), ROLL_VAR, PITCH_VAR, G);

      // ...but the robot is really, statically, rolled: a = Rᵀ(0,0,g), ω = 0.
      RotationMatrix trueRotation = new RotationMatrix();
      trueRotation.setYawPitchRoll(0.0, 0.0, roll);
      Vector3D specificForce = new Vector3D();
      trueRotation.inverseTransform(new Vector3D(0.0, 0.0, G), specificForce);

      settleGravityReference(updater, specificForce);
      updater.assemble(state, specificForce);

      // Residual = ĝ_ref − R̂ᵀe_z = (0, sin(roll), cos(roll)−1): full, undiminished tilt information.
      assertEquals(0.0, updater.getResidual().get(0, 0), 1.0e-6, "residual x");
      assertEquals(Math.sin(roll), updater.getResidual().get(1, 0), 1.0e-4, "residual y must carry the FULL static tilt");
      assertEquals(Math.cos(roll) - 1.0, updater.getResidual().get(2, 0), 1.0e-4, "residual z");
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
