package us.ihmc.stateEstimation.invariant_estimator;

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

   /** |g| (m/s²), used only by the gravity-leveling quasi-static gate. */
   private static final double GRAVITY_MAGNITUDE = 9.81;
   /** σ_tilt² default for the gravity-leveling measurement (rad²); ~ (0.05 rad ≈ 2.9°)². TODO(retune) vs NIS. */
   private static final double DEFAULT_TILT_MEASUREMENT_VARIANCE = 2.5e-3;

   /**
    * Builds a filter with the given contact count and continuous process-noise variances, wiring the
    * state, propagator, and updater (plus contact subpiece) to a consistent size.
    *
    * @param numberOfContacts the number of contact columns N (≥ 0).
    * @param gyroVariance     continuous angular-velocity noise variance σ_ω² (rad²/s).
    * @param accelVariance    continuous specific-force noise variance σ_a² (m²/s³).
    * @param contactVariance  continuous contact-slip noise variance σ_c² (m²/s).
    */
   public InvariantEKF(int numberOfContacts, double gyroVariance, double accelVariance, double contactVariance)
   {
      state = new InvariantState(numberOfContacts);
      propagator = new InvariantPropagator(numberOfContacts, gyroVariance, accelVariance, contactVariance);
      updater = new InvariantUpdater(state.getTangentSize());
      updater.setContactUpdater(new ContactUpdater(numberOfContacts));
      gravityUpdater = new GravityLevelingUpdater(state.getTangentSize(), DEFAULT_TILT_MEASUREMENT_VARIANCE, GRAVITY_MAGNITUDE);
   }

   /**
    * Static constructor: builds and wires an {@link InvariantEKF}. Equivalent to the constructor, provided as
    * the recommended entry point and the natural place to add named construction variants later.
    *
    * @param numberOfContacts the number of contact columns N (≥ 0).
    * @param gyroVariance     continuous angular-velocity noise variance σ_ω² (rad²/s).
    * @param accelVariance    continuous specific-force noise variance σ_a² (m²/s³).
    * @param contactVariance  continuous contact-slip noise variance σ_c² (m²/s).
    * @return the wired filter.
    */
   public static InvariantEKF create(int numberOfContacts, double gyroVariance, double accelVariance, double contactVariance)
   {
      return new InvariantEKF(numberOfContacts, gyroVariance, accelVariance, contactVariance);
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
      updater.update(state, contactIndex, bodyMeasurement, bodyMeasurementCovariance, false); // learned module not wired yet
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
   public boolean isGravityQuasiStatic(Vector3DReadOnly specificForceBody, Vector3DReadOnly angularVelocity, double accelToleranceRatio, double gyroThreshold)
   {
      return gravityUpdater.isQuasiStatic(specificForceBody, angularVelocity, accelToleranceRatio, gyroThreshold);
   }

   /** Sets σ_tilt² (rad²) for the gravity-leveling measurement noise. */
   public void setTiltMeasurementVariance(double tiltMeasurementVariance) { gravityUpdater.setTiltMeasurementVariance(tiltMeasurementVariance); }

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
