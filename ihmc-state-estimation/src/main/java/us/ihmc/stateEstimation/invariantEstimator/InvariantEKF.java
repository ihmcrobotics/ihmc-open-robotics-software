package us.ihmc.stateEstimation.invariantEstimator;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * World-centric right-invariant EKF on SE_k(3): the orchestrator that ties together the state,
 * propagation, and correction pieces of this package.
 *
 * <p>This class is pure wiring — it owns an {@link InvariantState}, an {@link InvariantPropagator}, and
 * an {@link InvariantUpdater} (with a {@link ContactUpdater} subpiece installed), all built from a single
 * contact count so their sizes are guaranteed consistent. It forwards calls in the right order and adds
 * no estimation logic of its own:
 * <ul>
 *   <li>{@link #predict} advances the state with the IMU (propagation step).</li>
 *   <li>{@link #update} corrects the state with one contact forward-kinematics measurement.</li>
 *   <li>the getters expose the current estimate.</li>
 * </ul>
 * The caller decides which contacts to update each tick; contact-lifecycle bookkeeping (active set,
 * touchdown anchoring, lift-off) is intentionally left to a higher layer. Biases are excluded — an
 * upstream joint-space EKF owns those, matching {@link InvariantState}.</p>
 */
public class InvariantEKF
{
   private final InvariantState state;
   private final InvariantPropagator propagator;
   private final InvariantUpdater updater;
   private final GravityLevelingUpdater gravityUpdater;

   /** σ_roll² default for the gravity-leveling measurement (rad²); ~ (0.05 rad ≈ 2.9°)². Trust the accel for roll. */
   private static final double DEFAULT_ROLL_MEASUREMENT_VARIANCE = 2.5e-3;
   /** σ_pitch² default (rad²); ~ (0.44 rad ≈ 25°)². Distrust the accel for pitch — fore-aft accel fakes pitch tilt
    *  and leans the base forward. Finite (not off) so pitch still levels slowly and the tilt diagnostic stays live. */
   private static final double DEFAULT_PITCH_MEASUREMENT_VARIANCE = 1.9e-1;

   /**
    * Builds a filter with the given contact count and continuous process-noise variances, wiring the
    * state, propagator, and updater (plus contact subpiece) to a consistent size.
    *
    * @param numberOfContacts the number of contact columns N (≥ 0).
    * @param gyroVariance     continuous angular-velocity noise variance σ_ω² (rad²/s).
    * @param accelVariance    continuous specific-force noise variance σ_a² (m²/s³).
    * @param contactVariance  continuous contact-slip noise variance σ_c² (m²/s).
    * @param gravitationalAcceleration the process gravity (m/s²); sign not considered. One value feeds both the
    *                                  propagator (0, 0, −|g|) and the quasi-static gate (|g|), so they cannot disagree.
    */
   public InvariantEKF(int numberOfContacts, double gyroVariance, double accelVariance, double contactVariance, double gravitationalAcceleration)
   {
      this(numberOfContacts, gyroVariance, accelVariance, contactVariance, gravitationalAcceleration, false);
   }

   /**
    * @param withGyroBias carries a body-frame gyro bias state (see {@link InvariantState}); inactive until
    *                     {@link #setGyroBiasEstimation} turns it on.
    */
   public InvariantEKF(int numberOfContacts,
                       double gyroVariance,
                       double accelVariance,
                       double contactVariance,
                       double gravitationalAcceleration,
                       boolean withGyroBias)
   {
      state = new InvariantState(numberOfContacts, withGyroBias);
      propagator = new InvariantPropagator(numberOfContacts, gyroVariance, accelVariance, contactVariance, gravitationalAcceleration, withGyroBias);
      updater = new InvariantUpdater(state.getTangentSize());
      updater.setContactUpdater(new ContactUpdater(numberOfContacts));
      gravityUpdater = new GravityLevelingUpdater(state.getTangentSize(),
                                                  DEFAULT_ROLL_MEASUREMENT_VARIANCE,
                                                  DEFAULT_PITCH_MEASUREMENT_VARIANCE,
                                                  Math.abs(gravitationalAcceleration));
   }

   /**
    * Static constructor: builds and wires an {@link InvariantEKF}. Equivalent to the constructor, provided as
    * the recommended entry point and the natural place to add named construction variants later.
    *
    * @param numberOfContacts the number of contact columns N (≥ 0).
    * @param gyroVariance     continuous angular-velocity noise variance σ_ω² (rad²/s).
    * @param accelVariance    continuous specific-force noise variance σ_a² (m²/s³).
    * @param contactVariance  continuous contact-slip noise variance σ_c² (m²/s).
    * @param gravitationalAcceleration the process gravity (m/s²); sign not considered.
    * @return the wired filter.
    */
   public static InvariantEKF create(int numberOfContacts, double gyroVariance, double accelVariance, double contactVariance, double gravitationalAcceleration)
   {
      return new InvariantEKF(numberOfContacts, gyroVariance, accelVariance, contactVariance, gravitationalAcceleration);
   }

   /**
    * Initializes the estimate: sets X = (rotation, baseVelocity, basePosition, contactPositions) and the
    * covariance P.
    *
    * @param rotation         the initial base orientation R. Not modified.
    * @param baseVelocity     the initial base velocity v. Not modified.
    * @param basePosition     the initial base position p. Not modified.
    * @param contactPositions the initial contact positions, length N (may be empty/null only if N = 0). Not modified.
    * @param initialCovariance the initial covariance P (m×m, m = 9 + 3N). Copied in. Not modified.
    */
   public void initialize(RotationMatrixReadOnly rotation,
                          Tuple3DReadOnly baseVelocity,
                          Tuple3DReadOnly basePosition,
                          Tuple3DReadOnly[] contactPositions,
                          DMatrixRMaj initialCovariance)
   {
      int numberOfContacts = state.getNumberOfContacts();
      int providedContacts = contactPositions == null ? 0 : contactPositions.length;
      if (providedContacts != numberOfContacts)
         throw new IllegalArgumentException("expected" + numberOfContacts + " contact positions, got " + providedContacts);

      int m = state.getTangentSize();
      if (initialCovariance.getNumRows() != m || initialCovariance.getNumCols() != m)
         throw new IllegalArgumentException("initialCovariance muhst be " + m + "x" + m + ", was " + initialCovariance.getNumRows() + "x" + initialCovariance.getNumCols());

      state.setRotation(rotation);
      state.setBaseVelocity(baseVelocity);
      state.setBasePosition(basePosition);
      for (int i = 0; i < numberOfContacts; i++)
         state.setContactPosition(i, contactPositions[i]);

      state.getCovariance().set(initialCovariance);
   }

   /**
    * Propagation step: advances the state forward by Δt using the bias-corrected IMU readings.
    *
    * @param angularVelocity    bias-corrected body-frame ω. Not modified.
    * @param linearAcceleration bias-corrected body-frame specific force a. Not modified.
    * @param dt                 the timestep Δt (> 0).
    */
   public void predict(Vector3DReadOnly angularVelocity, Vector3DReadOnly linearAcceleration, double dt)
   {
      propagator.predict(state, angularVelocity, linearAcceleration, dt);
   }

   /**
    * Retunes contact i's continuous slip-noise variance σ_{c,i}² used by the propagation step. Soft
    * contact handling calls this each tick to inflate a swing foot's anchor noise and restore it in
    * stance; see {@link InvariantPropagator#setContactSlipVariance}.
    *
    * @param contactIndex the contact index i in [0, N).
    * @param variance     the continuous slip variance σ_{c,i}² (m²/s), ≥ 0.
    */
   public void setContactSlipVariance(int contactIndex, double variance)
   {
      propagator.setContactSlipVariance(contactIndex, variance);
   }

   /**
    * Correction step: applies one contact forward-kinematics measurement.
    *
    * @param contactIndex              the contact index i in [0, N).
    * @param bodyMeasurement           the body-frame forward-kinematics measurement y = h_Cᵢ(q). Not modified.
    * @param bodyMeasurementCovariance the body-frame measurement covariance Nᵢ = J_Cᵢ·Σ_q·J_Cᵢᵀ (3×3). Not modified.
    */
   public void update(int contactIndex, Tuple3DReadOnly bodyMeasurement, Matrix3DReadOnly bodyMeasurementCovariance)
   {
      updater.update(state, contactIndex, bodyMeasurement, bodyMeasurementCovariance, false, contactNISGate); // learned module not wired yet
   }

   /** NIS above which a contact update's R is inflated (see {@link InvariantUpdater}); NaN disables. */
   private double contactNISGate = Double.NaN;

   public void setContactNISGate(double threshold)
   {
      contactNISGate = threshold;
   }

   private final us.ihmc.euclid.matrix.RotationMatrix reseedRotation = new us.ihmc.euclid.matrix.RotationMatrix();
   private final us.ihmc.euclid.tuple3D.Vector3D reseedPosition = new us.ihmc.euclid.tuple3D.Vector3D();
   private final us.ihmc.euclid.tuple3D.Vector3D reseedContactPosition = new us.ihmc.euclid.tuple3D.Vector3D();
   private final us.ihmc.euclid.tuple3D.Vector3D reseedRotatedMeasurement = new us.ihmc.euclid.tuple3D.Vector3D();
   private final us.ihmc.euclid.matrix.Matrix3D reseedRotatedCovariance = new us.ihmc.euclid.matrix.Matrix3D();

   /**
    * Touchdown anchor re-seed (H4 Phase 2, 2026-07-16 derivation note): redefines contact i in the
    * estimator's current gauge, d̂ᵢ ← p̂ + R̂·y, and makes P consistent by cloning the base-position
    * row/column into the contact block and adding the FK measurement covariance on its diagonal:
    * P_{dᵢ·} ← P_{p·}, P_{dᵢdᵢ} ← P_{pp} + R̂·Nᵢ·R̂ᵀ. This zeroes the contact residual and the
    * rotation/velocity gain rows (K_θ = (P_{θp}−P_{θdᵢ})S⁻¹ = 0), so the incoming foot releases none
    * of the swing-accumulated geometric discrepancy into the base. PSD-preserving by construction
    * (congruence + PSD addition). PLACEHOLDER for a learned contact model (ContactNet) which will
    * replace the binary full-trust with a learned blend at this same seam.
    *
    * @return the pre-reseed residual norm |R̂·y − (d̂ᵢ − p̂)| — the geometric discrepancy absorbed
    *         (snap avoided), for logging.
    */
   public double reseedContact(int contactIndex, Tuple3DReadOnly bodyMeasurement, Matrix3DReadOnly bodyMeasurementCovariance)
   {
      state.getRotation(reseedRotation);
      state.getBasePosition(reseedPosition);
      state.getContactPosition(contactIndex, reseedContactPosition);

      // pre-reseed residual r = R*y - (d - p), world frame
      us.ihmc.euclid.tuple3D.Vector3D rotated = reseedRotatedMeasurement; // a field: this runs on every touchdown
      reseedRotation.transform(bodyMeasurement, rotated);
      double rx = rotated.getX() - (reseedContactPosition.getX() - reseedPosition.getX());
      double ry = rotated.getY() - (reseedContactPosition.getY() - reseedPosition.getY());
      double rz = rotated.getZ() - (reseedContactPosition.getZ() - reseedPosition.getZ());
      double residualNorm = Math.sqrt(rx * rx + ry * ry + rz * rz);

      // d_i <- p + R*y
      rotated.add(reseedPosition);
      state.setContactPosition(contactIndex, rotated);

      // P: clone base-position row then column into the contact block (after both, P[d,d] = P[p,p]
      // and the block stays symmetric), then add the rotated measurement covariance on the diagonal.
      DMatrixRMaj covariance = state.getCovariance();
      int m = covariance.getNumRows();
      int pIdx = state.basePositionTangentIndex();
      int dIdx = state.contactTangentIndex(contactIndex);
      for (int a = 0; a < 3; a++)
         for (int col = 0; col < m; col++)
            covariance.set(dIdx + a, col, covariance.get(pIdx + a, col));
      for (int row = 0; row < m; row++)
         for (int a = 0; a < 3; a++)
            covariance.set(row, dIdx + a, covariance.get(row, pIdx + a));

      reseedRotatedCovariance.set(bodyMeasurementCovariance);
      reseedRotation.transform(reseedRotatedCovariance); // R * N * R^T
      for (int a = 0; a < 3; a++)
         for (int b = 0; b < 3; b++)
            covariance.add(dIdx + a, dIdx + b, reseedRotatedCovariance.getElement(a, b));

      return residualNorm;
   }

   /**
    * @return the Normalized Innovation Squared rᵀ·S⁻¹·r from the most recent {@link #update} call, or
    *         {@code NaN} if none has run. Read immediately after an {@code update} to get that
    *         measurement's consistency statistic; a later {@code update} overwrites it.
    */
   public double getLastNormalizedInnovationSquared()
   {
      return updater.getNormalizedInnovationSquared();
   }

   /**
    * Assembles the gravity-leveling (tilt) measurement from the current state and the bias-corrected body-frame
    * specific force, and refreshes the tilt-error diagnostic — WITHOUT applying the correction. Call this every
    * tick so the tilt-error diagnostic ({@link #getGravityTiltErrorAngle()} etc.) tracks pitch/roll error
    * continuously; then gate on {@link #isGravityQuasiStatic} and call {@link #applyGravityLeveling()} only when
    * the accelerometer is a trustworthy gravity reference.
    *
    * @param specificForceBody the bias-corrected body-frame specific force a (the vector fed to {@link #predict}). Not modified.
    */
   public void assembleGravityLeveling(Vector3DReadOnly specificForceBody)
   {
      gravityUpdater.assemble(state, specificForceBody);
   }

   /** Applies the gravity-leveling correction assembled by the last {@link #assembleGravityLeveling} (X and P updated in place). */
   public void applyGravityLeveling()
   {
      updater.update(state, gravityUpdater.getMeasurementJacobian(), gravityUpdater.getResidual(), gravityUpdater.getMeasurementCovariance());
   }

   /** @return true if the accelerometer can be trusted as a gravity reference this tick (see {@link GravityLevelingUpdater#isQuasiStatic}). */
   public boolean isGravityQuasiStatic(Vector3DReadOnly specificForceBody, Vector3DReadOnly rawAngularVelocity,
                                       double accelToleranceRatio, double gyroThreshold, double horizontalAccelThreshold)
   {
      return gravityUpdater.isQuasiStatic(specificForceBody, rawAngularVelocity, accelToleranceRatio, gyroThreshold, horizontalAccelThreshold);
   }

   // Kinematic base-velocity update: preallocated, the measurement size is always 3.
   private DMatrixRMaj velocityJacobian = null;
   private final DMatrixRMaj velocityResidual = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj velocityNoise = new DMatrixRMaj(3, 3);
   private final us.ihmc.euclid.matrix.RotationMatrix velocityRotation = new us.ihmc.euclid.matrix.RotationMatrix();
   private final us.ihmc.euclid.tuple3D.Vector3D velocityEstimate = new us.ihmc.euclid.tuple3D.Vector3D();
   private final us.ihmc.euclid.tuple3D.Vector3D velocityMeasurementWorld = new us.ihmc.euclid.tuple3D.Vector3D();
   private final us.ihmc.euclid.matrix.Matrix3D velocityNoiseWorld = new us.ihmc.euclid.matrix.Matrix3D();

   /**
    * Correction step: a body-frame measurement of the base velocity, {@code y = Rᵀv}. On a stance foot leg
    * kinematics gives it directly, {@code y = −(ω × r + ṙ)} with r the sole in the body frame -- the measurement
    * the kinematics-based estimator lives on, which keeps velocity from running away between the position-only
    * contact corrections.
    * <p>
    * Residual {@code R̂·y − v̂}. With {@code X̂ = exp(ξ)·X}, {@code R̂ ≈ (I + ξ_R×)R} and {@code v̂ ≈ (I + ξ_R×)v + ξ_v},
    * so the residual is {@code −ξ_v} to first order -- the rotation error cancels -- and H is −I on the velocity
    * block, zero elsewhere. The noise is rotated to world, {@code R̂·N·R̂ᵀ}.
    *
    * @param bodyVelocity           y, the base velocity in the body frame. Not modified.
    * @param bodyVelocityCovariance N, its 3×3 covariance in the body frame. Not modified.
    */
   public void updateBodyVelocity(Tuple3DReadOnly bodyVelocity, Matrix3DReadOnly bodyVelocityCovariance)
   {
      updateBodyVelocity(bodyVelocity, bodyVelocityCovariance, null);
   }

   private final us.ihmc.euclid.matrix.Matrix3D velocityBiasColumns = new us.ihmc.euclid.matrix.Matrix3D();

   /**
    * As {@link #updateBodyVelocity(Tuple3DReadOnly, Matrix3DReadOnly)} for a measurement built from the bias-corrected
    * rate, {@code y = −((ω_m − b̂_g) × r + ṙ)}. With {@code δb = b̂_g − b_g} that is {@code y = Rᵀv − [r]ₓ δb}, so
    * with an active gyro bias state the residual gains {@code −R̂[r]ₓ δb} and H the bias columns {@code −R̂[r]ₓ}.
    *
    * @param leverArm r, the stationary point in the body frame; null (or no active bias state) leaves H as before.
    */
   public void updateBodyVelocity(Tuple3DReadOnly bodyVelocity, Matrix3DReadOnly bodyVelocityCovariance, Tuple3DReadOnly leverArm)
   {
      int m = state.getTangentSize();
      if (velocityJacobian == null || velocityJacobian.getNumCols() != m)
      {
         velocityJacobian = new DMatrixRMaj(3, m);
         int v = state.baseVelocityTangentIndex();
         for (int i = 0; i < 3; i++)
            velocityJacobian.set(i, v + i, -1.0);
      }
      state.getRotation(velocityRotation);
      state.getBaseVelocity(velocityEstimate);
      velocityRotation.transform(bodyVelocity, velocityMeasurementWorld);
      velocityResidual.set(0, 0, velocityMeasurementWorld.getX() - velocityEstimate.getX());
      velocityResidual.set(1, 0, velocityMeasurementWorld.getY() - velocityEstimate.getY());
      velocityResidual.set(2, 0, velocityMeasurementWorld.getZ() - velocityEstimate.getZ());
      velocityNoiseWorld.set(bodyVelocityCovariance);
      velocityRotation.transform(velocityNoiseWorld); // R N Rᵀ
      velocityNoiseWorld.get(velocityNoise);
      int b = state.gyroBiasTangentIndex();
      if (b >= 0)
      {
         boolean couple = leverArm != null && propagator.isGyroBiasActive();
         if (couple)
         {
            // −R̂·[r]ₓ
            velocityBiasColumns.setToTildeForm(leverArm);
            velocityBiasColumns.preMultiply(velocityRotation);
            velocityBiasColumns.scale(-1.0);
         }
         for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
               velocityJacobian.set(r, b + c, couple ? velocityBiasColumns.getElement(r, c) : 0.0);
      }
      updater.update(state, velocityJacobian, velocityResidual, velocityNoise);
   }

   private final DMatrixRMaj biasJacobian = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj biasResidual = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj biasNoise = new DMatrixRMaj(3, 3);

   /**
    * Direct measurement of the body-frame gyro bias (e.g. the mean raw rate over a standing rest): residual
    * {@code b̂_g − z ≈ δb}, H = I on the bias block. No-op without an active bias state.
    *
    * @param measuredBias z, body frame (rad/s). Not modified.
    * @param variance     per-axis variance of z ((rad/s)²).
    */
   public void updateGyroBias(Tuple3DReadOnly measuredBias, double variance)
   {
      int b = state.gyroBiasTangentIndex();
      if (b < 0 || !propagator.isGyroBiasActive())
         return;
      int m = state.getTangentSize();
      biasJacobian.reshape(3, m);
      biasJacobian.zero();
      for (int i = 0; i < 3; i++)
         biasJacobian.set(i, b + i, 1.0);
      us.ihmc.euclid.tuple3D.Vector3D estimate = state.getGyroBias();
      biasResidual.set(0, 0, estimate.getX() - measuredBias.getX());
      biasResidual.set(1, 0, estimate.getY() - measuredBias.getY());
      biasResidual.set(2, 0, estimate.getZ() - measuredBias.getZ());
      biasNoise.zero();
      for (int i = 0; i < 3; i++)
         biasNoise.set(i, i, variance);
      updater.update(state, biasJacobian, biasResidual, biasNoise);
   }

   /** Turns the gyro bias state on or off and sets its random walk σ_b² ((rad/s)²/s); see {@link InvariantPropagator#setGyroBias}. */
   public void setGyroBiasEstimation(boolean active, double randomWalkVariance)
   {
      propagator.setGyroBias(active, randomWalkVariance);
   }

   public boolean isGyroBiasEstimationActive()
   {
      return propagator.isGyroBiasActive();
   }

   /** @return the live body-frame gyro bias estimate (zero without the bias state). */
   public us.ihmc.euclid.tuple3D.Vector3D getGyroBias()
   {
      return state.getGyroBias();
   }

   /**
    * Advances the quasi-static gate's sensor-only gravity reference. Call once per tick, before
    * {@link #isGravityQuasiStatic}, with RAW sensor values. See {@link GravityLevelingUpdater#updateGravityReference}.
    */
   public void updateGravityReference(Vector3DReadOnly specificForceBody, Vector3DReadOnly rawAngularVelocity, double dt)
   {
      gravityUpdater.updateGravityReference(specificForceBody, rawAngularVelocity, dt);
   }

   /** The sensor-only gravity reference used by the quasi-static gate (unit, body frame). For tests/diagnostics. */
   public Vector3DReadOnly getGravityReference() { return gravityUpdater.getGravityReference(); }

   /** Sets σ_roll² and σ_pitch² (rad²) for the (anisotropic) gravity-leveling measurement noise. */
   public void setGravityMeasurementVariances(double rollVariance, double pitchVariance) { gravityUpdater.setMeasurementVariances(rollVariance, pitchVariance); }

   /** Isotropic convenience: sets σ_roll² = σ_pitch² for the gravity-leveling measurement noise. */
   public void setTiltMeasurementVariance(double tiltMeasurementVariance) { gravityUpdater.setTiltMeasurementVariance(tiltMeasurementVariance); }

   /** Gates the gravity-leveling PITCH observation on/off (roll is always observed). See {@link GravityLevelingUpdater#setPitchObservable}. */
   public void setGravityPitchObservable(boolean pitchObservable) { gravityUpdater.setPitchObservable(pitchObservable); }

   /** Total tilt error angle (rad) between measured and predicted gravity direction — valid every tick, even when the update is gated off. */
   public double getGravityTiltErrorAngle() { return gravityUpdater.getTiltErrorAngle(); }
   /** Body-frame roll component (rad) of the tilt error. */
   public double getGravityTiltErrorRoll()  { return gravityUpdater.getTiltErrorRoll(); }
   /** Body-frame pitch component (rad) of the tilt error. */
   public double getGravityTiltErrorPitch() { return gravityUpdater.getTiltErrorPitch(); }

   // Innovation-covariance conditioning diagnostics from the most recent update (contact or gravity).
   public double getLastConditionProxy() { return updater.getConditionProxy(); }
   public double getLastMinSDiagonal()   { return updater.getMinSDiagonal(); }
   public double getLastMaxSDiagonal()   { return updater.getMaxSDiagonal(); }
   public double getLastResidualNorm()   { return updater.getResidualNorm(); }
   public boolean wasLastUpdateApplied() { return updater.wasLastUpdateApplied(); }
   public int getUpdateGateSkipCount()   { return updater.getGateSkipCount(); }
   public double getLastInnovationInflation() { return updater.getInnovationInflation(); }
   public int getInnovationInflatedCount()    { return updater.getInnovationInflatedCount(); }
   /** trace(H·P·Hᵀ) of the most recent update's S — the state-covariance share. */
   public double getLastHPHtTrace()      { return updater.getLastHPHtTrace(); }
   /** trace(R) of the most recent update's S — the measurement-noise share (post-inflation). */
   public double getLastMeasurementNoiseTrace() { return updater.getLastMeasurementNoiseTrace(); }
   /** |δθ| (rad) of the most recent applied correction — H4 anchor-transient diagnostic. */
   public double getLastCorrectionRotationNorm() { return updater.getLastCorrectionRotationNorm(); }
   /** |δv| (m/s) of the most recent applied correction. */
   public double getLastCorrectionVelocityNorm() { return updater.getLastCorrectionVelocityNorm(); }
   /** |δp| (m) of the most recent applied correction. */
   public double getLastCorrectionPositionNorm() { return updater.getLastCorrectionPositionNorm(); }

   /** @return the number of contact columns N. */
   public int getNumberOfContacts()
   {
      return state.getNumberOfContacts();
   }

   /** @return the live estimate state X and P. Mutating it mutates the filter. */
   public InvariantState getState()
   {
      return state;
   }

   /**
    * Packs the current base orientation R.
    *
    * @param rotationToPack the rotation to pack the result into. Modified.
    */
   public void getRotation(RotationMatrixBasics rotationToPack)
   {
      state.getRotation(rotationToPack);
   }

   /**
    * Packs the current base velocity v.
    *
    * @param velocityToPack the vector to pack the result into. Modified.
    */
   public void getBaseVelocity(Vector3DBasics velocityToPack)
   {
      state.getBaseVelocity(velocityToPack);
   }

   /**
    * Overwrites the base orientation R of the state mean. Used by an external yaw-seeding correction (the
    * yaw direction is unobservable in the right-invariant contact filter); a mean-only complementary
    * nudge that intentionally leaves the covariance untouched.
    *
    * @param rotation the new base orientation R. Not modified.
    */
   public void setRotation(RotationMatrixReadOnly rotation)
   {
      state.setRotation(rotation);
   }

   /**
    * Packs the current base position p.
    *
    * @param positionToPack the vector to pack the result into. Modified.
    */
   public void getBasePosition(Vector3DBasics positionToPack)
   {
      state.getBasePosition(positionToPack);
   }

   /**
    * Packs the current position of contact i.
    *
    * @param contactIndex   the contact index i in [0, N).
    * @param positionToPack the vector to pack the result into. Modified.
    */
   public void getContactPosition(int contactIndex, Vector3DBasics positionToPack)
   {
      state.getContactPosition(contactIndex, positionToPack);
   }
}
