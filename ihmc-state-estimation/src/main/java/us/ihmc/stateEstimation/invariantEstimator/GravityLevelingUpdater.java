package us.ihmc.stateEstimation.invariantEstimator;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Accelerometer gravity-leveling (tilt) measurement for the right-invariant EKF — the roll/pitch
 * observation the contact update does NOT provide.
 *
 * <p><b>Why this exists.</b> The only other measurement in this filter is the contact forward-kinematics
 * update, whose Jacobian has a ZERO rotation block ({@link ContactUpdater}) — it never directly observes
 * orientation. Roll/pitch are otherwise only propagated open-loop from the (bias-corrected) gyro, so any
 * residual gyro tilt-bias integrates straight into pelvis pitch/roll with nothing to pull it back (the
 * observed slow pitch drift). When the robot is quasi-static the accelerometer measures gravity direction,
 * which directly observes tilt — this class turns that into a proper invariant measurement update.</p>
 *
 * <p><b>Measurement model.</b> The body-frame specific force is {@code a = Rᵀ(a_world − g)}, so the normalized
 * specific force {@code f̂ = a/‖a‖} equals the world up-axis in body frame ONLY when {@code a_world ≈ 0}. For a
 * balancing biped that assumption is false in the worst possible way — {@code a_world} is the lateral CoM
 * acceleration the balance controller produces in response to this update's own output — so the measurement
 * fed to the filter is <b>{@code ĝ_ref}</b>, the gyro/accelerometer complementary gravity reference (see
 * {@link #updateGravityReference} and the block comment in {@link #assemble}), which carries true tilt at unity
 * gain and low-passes that artifact away. Everything below is written with {@code f̂} standing for that
 * measurement. The prediction is {@code ĝ_body = R̂ᵀ e_z}. With this package's right-invariant error
 * convention {@code R̂ = Exp(δφ)·R} (so {@code X̂ = exp(ξ)·X}):
 * <pre>
 *   f̂ = Rᵀ e_z = R̂ᵀ Exp(δφ) e_z ≈ ĝ_body − R̂ᵀ[e_z]ₓ δφ
 *   residual = f̂ − ĝ_body ≈ −R̂ᵀ[e_z]ₓ δφ = H·ξ
 * </pre>
 * so the 3×m Jacobian has its δφ-block {@code H_φ = −R̂ᵀ[e_z]ₓ = [ −R̂ᵀe_y | +R̂ᵀe_x | 0 ]} and zeros
 * everywhere else. {@code H_φ} is rank 2 with its null direction along gravity — the update therefore
 * corrects roll/pitch and is <b>yaw-safe by construction</b> (yaw is unobservable and left untouched). Feeding
 * {@code (H, residual, R)} to {@link InvariantUpdater#update} performs the correction; the null (yaw) direction
 * is regularized by {@code R}, so {@code S = HPHᵀ + R} stays invertible.</p>
 *
 * <p><b>Anisotropic (roll-trusting) measurement noise.</b> The quasi-static assumption {@code a_world ≈ 0} is
 * violated most along the PITCH axis: fore-aft locomotion accelerates the CoM fore/aft, so that horizontal
 * specific force masquerades as pitch tilt and, trusted, leans the base forward. Lateral (roll) accelerations
 * are smaller and roughly symmetric over a stride. So instead of an isotropic {@code R = σ_tilt²·I₃} we trust
 * roll and distrust pitch. The direction to distrust is the residual produced by a <b>body</b>-pitch error:
 * from {@code f̂ = Exp(δθ)·ĝ_body} the residual is {@code r = δθ × ĝ_body}, so a pure body-pitch error
 * {@code δθ = θ·e_y} lands along {@code û_p = normalize(e_y × ĝ_body)}, and
 * <pre>
 *   R = σ_roll² I₃ + (σ_pitch² − σ_roll²) û_p û_pᵀ
 * </pre>
 * (NOT {@code R̂ᵀe_x} — that is the residual direction of a WORLD-Y rotation. The two agree only at zero yaw; at
 * yaw ψ they differ by exactly ψ, aiming the pitch distrust at the wrong axis. See jointkf_bias_observability.md §4.)
 * which is PSD and still yaw-safe ({@code ĝ = R̂ᵀe_z} carries {@code σ_roll²}, regularizing the null direction).
 * {@code σ_pitch² = σ_roll²} recovers the isotropic update; a large {@code σ_pitch²} recovers a pure roll update.
 * When {@link #setPitchObservable(boolean) pitch is gated out} (e.g. single-support), {@code σ_pitch²} jumps to
 * a large disabled value, leaving the roll correction untouched.</p>
 *
 * <p>Allocation-free after construction. Also computes a per-tick tilt-error diagnostic (angle, and body-frame
 * roll/pitch components) that is valid EVERY tick — even when the quasi-static gate blocks the actual update —
 * so it can be watched on hardware as a direct "is pitch wrong" signal, unlike the sim-only ground-truth
 * comparator.</p>
 */
public class GravityLevelingUpdater
{
   private static final int MEASUREMENT_SIZE = 3;

   private final int tangentSize;
   private final double gravityMagnitude;

   /** σ_pitch² substituted when the pitch observation is gated out (see {@link #setPitchObservable}); large so
    *  pitch is effectively unobserved while S stays well-conditioned. */
   private static final double PITCH_DISABLED_VARIANCE = 1.0e6;

   private final DMatrixRMaj measurementJacobian;      // H (3×m)
   private final DMatrixRMaj residual = new DMatrixRMaj(MEASUREMENT_SIZE, 1);
   private final DMatrixRMaj measurementCovariance = new DMatrixRMaj(MEASUREMENT_SIZE, MEASUREMENT_SIZE); // anisotropic R, rebuilt each assemble()

   private double rollMeasurementVariance;             // σ_roll²  (rad², on the roll-informing residual direction)
   private double pitchMeasurementVariance;            // σ_pitch² (rad², on the pitch-informing residual direction)
   private boolean pitchObservable = true;             // when false, σ_pitch² → PITCH_DISABLED_VARIANCE (roll-only)

   /**
    * Time constant τ (s) of the gravity reference ĝ_ref (see {@link #updateGravityReference}), which is BOTH the
    * quasi-static gate's reference and — since the roll-sway fix — the measurement this update actually corrects
    * against.
    *
    * <p><b>Why τ is 5 s and not 0.5 s.</b> Linearizing the complementary filter about upright, its roll channel is
    * <pre>
    *   ĝ_ref = HPF_τ(∫ω_raw) + LPF_τ(f̂) = φ + HPF_τ(b_raw·t) + LPF_τ(ÿ/g)
    * </pre>
    * i.e. the TRUE tilt φ passes with <b>unity gain at every frequency</b> (the gyro carries it), while the two
    * error terms are shaped: the raw-gyro bias contributes a constant b_raw·τ, and the lateral-acceleration
    * artifact ÿ/g is low-passed with corner 1/τ. τ is therefore the single knob that trades <i>static</i>
    * raw-gyro-bias offset against <i>dynamic</i> rejection of the robot's own acceleration — and it costs nothing
    * in true-tilt tracking.
    *
    * <p>τ must be chosen so the artifact is rejected at the robot's LATERAL BALANCE frequency, ω_b ≈ 3.1 rad/s
    * (0.49 Hz, measured — Alex002 log 20260712_185914). Rejection there is 1/√(1+(ω_b τ)²):
    * τ = 0.5 s gives only 0.55 (45% rejected — useless), τ = 5 s gives 0.065 (15× rejected).
    * The cost is a static offset b_raw·τ = 0.001 rad/s · 5 s ≈ 5 mrad = 0.3°, using the RAW gyro whose true bias
    * is small (0.001 rad/s, measured on a motionless robot) — not the upstream JointKF bias, which is not.
    */
   private static final double GRAVITY_REFERENCE_TIME_CONSTANT = 5.0;

   // Pre-allocated scratch (allocation-free hot path).
   private final RotationMatrix rotation = new RotationMatrix();
   private final Vector3D predictedGravityBody = new Vector3D();     // ĝ_body = R̂ᵀ e_z
   private final Vector3D measuredGravityBody = new Vector3D();      // the update's measurement: ĝ_ref (see assemble)
   private final Vector3D rawMeasuredGravityBody = new Vector3D();   // f̂ = a/‖a‖ — DIAGNOSTIC ONLY (tilt-error vars)
   private final Vector3D rTransposeEx = new Vector3D();          // R̂ᵀ e_x
   private final Vector3D rTransposeEy = new Vector3D();          // R̂ᵀ e_y
   private final Vector3D pitchDirection = new Vector3D();        // û_p = normalize(e_y × ĝ_body), body-pitch residual dir
   private final Vector3D tiltErrorVector = new Vector3D();       // ĝ_body × f̂ (≈ body-frame roll/pitch error)
   private final Vector3D horizontalAccel = new Vector3D();       // a − (a·ĝ_ref)ĝ_ref, for the quasi-static gate

   /**
    * Gate-only gravity reference ĝ_ref: a unit body-frame estimate of the world up-axis maintained by a
    * complementary filter over the RAW gyro and the accelerometer. It deliberately reads NO filter state —
    * see {@link #isQuasiStatic} for why that is the whole point.
    */
   private final Vector3D gravityReferenceBody = new Vector3D();
   private final Vector3D referenceRate = new Vector3D();         // ω × ĝ_ref
   private boolean gravityReferenceInitialized = false;

   private static final Vector3D E_X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D E_Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D E_Z = new Vector3D(0.0, 0.0, 1.0);

   /**
    * Isotropic convenience: σ_roll² = σ_pitch² = {@code tiltMeasurementVariance}.
    *
    * @param tangentSize             the tangent/covariance dimension m (= 9 + 3N).
    * @param tiltMeasurementVariance σ_tilt² — measurement noise on the unit gravity-direction residual (rad²).
    * @param gravityMagnitude        |g| (m/s²), used only by the quasi-static gate.
    */
   public GravityLevelingUpdater(int tangentSize, double tiltMeasurementVariance, double gravityMagnitude)
   {
      this(tangentSize, tiltMeasurementVariance, tiltMeasurementVariance, gravityMagnitude);
   }

   /**
    * @param tangentSize              the tangent/covariance dimension m (= 9 + 3N).
    * @param rollMeasurementVariance  σ_roll²  — noise on the roll-informing residual direction (rad²).
    * @param pitchMeasurementVariance σ_pitch² — noise on the pitch-informing residual direction (rad²); make it
    *                                 larger than σ_roll² to distrust the accelerometer for pitch (fore-aft accel).
    * @param gravityMagnitude         |g| (m/s²), used only by the quasi-static gate.
    */
   public GravityLevelingUpdater(int tangentSize, double rollMeasurementVariance, double pitchMeasurementVariance, double gravityMagnitude)
   {
      this.tangentSize = tangentSize;
      this.rollMeasurementVariance = rollMeasurementVariance;
      this.pitchMeasurementVariance = pitchMeasurementVariance;
      this.gravityMagnitude = gravityMagnitude;
      this.measurementJacobian = new DMatrixRMaj(MEASUREMENT_SIZE, tangentSize);
   }

   /** Sets σ_roll² and σ_pitch² (rad²). R is rebuilt on the next {@link #assemble} (it depends on R̂). */
   public void setMeasurementVariances(double rollMeasurementVariance, double pitchMeasurementVariance)
   {
      this.rollMeasurementVariance = rollMeasurementVariance;
      this.pitchMeasurementVariance = pitchMeasurementVariance;
   }

   /** Isotropic convenience: sets σ_roll² = σ_pitch² = {@code tiltMeasurementVariance}. */
   public void setTiltMeasurementVariance(double tiltMeasurementVariance)
   {
      setMeasurementVariances(tiltMeasurementVariance, tiltMeasurementVariance);
   }

   /**
    * Gates the PITCH observation on/off without touching roll. When {@code false}, σ_pitch² is replaced by a
    * large disabled value on the next {@link #assemble}, so the update levels roll only (used e.g. to restrict
    * pitch leveling to double support). Roll is always observed.
    */
   public void setPitchObservable(boolean pitchObservable)
   {
      this.pitchObservable = pitchObservable;
   }

   /**
    * Assembles (H, residual, R) for the gravity-leveling measurement from the current state and the
    * bias-corrected body-frame specific force, and refreshes the tilt-error diagnostic. Allocation-free.
    *
    * @param state             the estimate state (read for R̂). Not modified.
    * @param specificForceBody the bias-corrected body-frame specific force a (same vector fed to predict). Not modified.
    */
   public void assemble(InvariantState state, Vector3DReadOnly specificForceBody)
   {
      state.getRotation(rotation);

      // ĝ_body = R̂ᵀ e_z (world up seen in body).
      rotation.inverseTransform(E_Z, predictedGravityBody);

      // Raw normalized specific force f̂ = a/‖a‖. This is the DIAGNOSTIC signal only — see below.
      rawMeasuredGravityBody.set(specificForceBody);
      double rawNorm = rawMeasuredGravityBody.norm();
      if (rawNorm > 1.0e-9)
         rawMeasuredGravityBody.scale(1.0 / rawNorm);

      // ---------------------------------------------------------------------------------------------------
      // The MEASUREMENT is ĝ_ref (the gyro/accel complementary gravity reference), NOT the instantaneous f̂.
      //
      // f̂ is not a measurement of gravity — it is a measurement of gravity PLUS the body's own acceleration:
      //   a = Rᵀ(a_world − g)  ⇒  f̂ ≈ ĝ_body + (Rᵀ a_world)/g.
      // For a balancing biped a_world is not noise: it is the lateral CoM acceleration ÿ that the balance
      // controller itself produces in response to the roll this very update publishes. That closes a POSITIVE
      // FEEDBACK loop through the controller:
      //     phantom roll in the estimate → controller torques to correct it → real lateral pelvis accel ÿ
      //     → f̂ tilts by ÿ/g (same sign) → MORE phantom roll.
      // Alex002 log 20260712_185914 (t=200-212 s, whole-body control, this update firing on 97% of ticks): the
      // roll ESTIMATE oscillated at 0.49 Hz with amplitude 1.17°, while the true roll (raw-gyro integration)
      // was only 0.19° — the accelerometer's apparent tilt was 6.5× the real tilt and was almost entirely
      // ÿ/g (ÿ ≈ 0.21 m/s², i.e. 2.3 cm of lateral sway). The oscillation grew (0.86° → 2.84° rms) until the
      // robot fell at t=226 s. Note 0.21 m/s² sails straight through the 0.5 m/s² horizontal-accel gate — the
      // gate cannot fix this, and tightening it only trades the limit cycle for open-loop drift.
      //
      // ĝ_ref (see updateGravityReference) is the same sensors combined correctly: the gyro carries the true
      // tilt at ALL frequencies (unity gain), and the accelerometer is low-passed with time constant τ, so the
      // ÿ/g artifact is attenuated by 1/√(1+(ωτ)²) — 15× at the 0.49 Hz balance mode. This removes the
      // artifact WITHOUT weakening the update's DC authority, which is what the pitch/roll drift correction
      // actually needs. (Detuning σ_roll² instead would give up both together: it lowers the leveling loop
      // bandwidth λ ∝ 1/σ², and λ is exactly the authority that holds the base against a residual gyro bias.)
      //
      // Gravity is DC. Any AC content in the specific-force DIRECTION is, by construction, not gravity.
      // ---------------------------------------------------------------------------------------------------
      if (gravityReferenceInitialized)
         measuredGravityBody.set(gravityReferenceBody);
      else
         measuredGravityBody.set(rawMeasuredGravityBody); // first tick, before the reference is seeded

      // residual = ĝ_ref − ĝ_body ≈ −R̂ᵀ[e_z]ₓ δφ.
      residual.set(0, 0, measuredGravityBody.getX() - predictedGravityBody.getX());
      residual.set(1, 0, measuredGravityBody.getY() - predictedGravityBody.getY());
      residual.set(2, 0, measuredGravityBody.getZ() - predictedGravityBody.getZ());

      // H_φ = −R̂ᵀ[e_z]ₓ = [ −R̂ᵀe_y | +R̂ᵀe_x | 0 ]; zeros on v, p, and all contact blocks.
      rotation.inverseTransform(E_X, rTransposeEx);
      rotation.inverseTransform(E_Y, rTransposeEy);
      measurementJacobian.reshape(MEASUREMENT_SIZE, tangentSize);
      measurementJacobian.zero();
      for (int r = 0; r < 3; r++)
      {
         measurementJacobian.set(r, 0, -elementOf(rTransposeEy, r)); // column δφ_x = −R̂ᵀe_y
         measurementJacobian.set(r, 1, +elementOf(rTransposeEx, r)); // column δφ_y = +R̂ᵀe_x
         // column δφ_z (index 2) stays zero → yaw unobserved.
      }

      // Anisotropic R = σ_roll² I₃ + (σ_pitch²_eff − σ_roll²) û_p û_pᵀ, with û_p the residual direction produced
      // by a BODY-pitch error. From f̂ = Exp(δθ)·ĝ_body, the residual is r = δθ × ĝ_body, so a pure body-pitch
      // error δθ = θ·e_y lands along û_p = normalize(e_y × ĝ_body). (The old û_p = R̂ᵀe_x is the residual
      // direction of a WORLD-Y rotation; the two coincide only at zero yaw — at yaw ψ they differ by exactly ψ,
      // aiming the pitch distrust at the wrong axis. See jointkf_bias_observability.md §4.)
      pitchDirection.cross(E_Y, predictedGravityBody);
      double pitchDirectionNorm = pitchDirection.norm();
      double effectivePitchVariance = pitchObservable ? pitchMeasurementVariance : PITCH_DISABLED_VARIANCE;
      // Degenerate when ĝ_body ∥ e_y (robot on its side): body pitch is not identifiable from gravity, so fall
      // back to isotropic R rather than inflating a meaningless direction.
      double pitchMinusRoll = pitchDirectionNorm > 1.0e-6 ? effectivePitchVariance - rollMeasurementVariance : 0.0;
      if (pitchDirectionNorm > 1.0e-6)
         pitchDirection.scale(1.0 / pitchDirectionNorm);
      for (int r = 0; r < 3; r++)
      {
         double ur = elementOf(pitchDirection, r);
         for (int c = 0; c < 3; c++)
            measurementCovariance.set(r, c, (r == c ? rollMeasurementVariance : 0.0) + pitchMinusRoll * ur * elementOf(pitchDirection, c));
      }

      // Diagnostic: tilt-error vector ĝ_body × f̂, deliberately against the RAW f̂ (not the ĝ_ref the update now
      // corrects against), so the published invariantGravityTiltError* keep their original meaning — "what is the
      // accelerometer saying about tilt RIGHT NOW, relative to the estimate". That is the signal that exposed the
      // roll sway (est + tiltErrorRoll reconstructs the accelerometer-implied roll), and it must stay observable
      // rather than be filtered into agreement with the update.
      tiltErrorVector.cross(predictedGravityBody, rawMeasuredGravityBody);
   }

   private static double elementOf(Vector3DReadOnly v, int index)
   {
      return index == 0 ? v.getX() : index == 1 ? v.getY() : v.getZ();
   }

   /**
    * Quasi-static gate: the accelerometer only measures gravity direction when linear acceleration is small
    * (‖a‖ ≈ |g|), the body is barely rotating (‖ω‖ small), AND the HORIZONTAL specific force is small. The last
    * condition is the important one: a ‖a‖-norm-only gate of ±5% still admits up to ~0.3|g| of horizontal accel
    * (≈18° of false tilt) whenever the vertical channel compensates, so the fore-aft accel that leans the base
    * forward slips through.
    *
    * <p><b>The horizontal term must NOT be referenced to the estimate — that is a self-latching bug.</b> This
    * gate previously resolved the horizontal component against {@code ĝ = R̂ᵀe_z}, the filter's own up-axis. For
    * a genuinely static robot the accelerometer reads {@code a = g·u_true}, so that quantity evaluates to
    * {@code ‖a_horiz‖ = g·sin θ} where <b>θ is the tilt error itself</b> — making the gate
    * {@code θ ≤ arcsin(0.5/9.81) = 0.051 rad}. The only tilt corrector in the filter therefore switched itself
    * OFF exactly when its own error exceeded 2.9°, and since the error then grows monotonically it could never
    * re-close. A corrector whose gate depends on its own error is unstable by construction; hardware log
    * 20260712_163634 latched at t=597 s and never leveled again (FINDINGS.md §F.3).
    *
    * <p>The horizontal term is now resolved against {@link #updateGravityReference ĝ_ref}, a gravity reference
    * built from the raw gyro and accelerometer alone, and {@code lowRotation} uses the RAW gyro. <b>No term in
    * this gate reads any filter state</b>, so it cannot latch. Known trade-off: a <i>sustained, constant</i>
    * horizontal acceleration with ‖a‖ ≈ |g| is eventually absorbed into ĝ_ref and would slip through — that
    * requires sliding feet or an accelerating vehicle (a constant-speed treadmill has zero acceleration), and is
    * strictly preferable to a corrector that latches off permanently.
    *
    * @param specificForceBody       the body-frame specific force a. Not modified.
    * @param rawAngularVelocity      the RAW body-frame angular velocity ω (no bias correction). Not modified.
    * @param accelToleranceRatio     allowed fractional deviation of ‖a‖ from |g| (e.g. 0.05 = ±5%).
    * @param gyroThreshold           max ‖ω‖ (rad/s) for "barely rotating".
    * @param horizontalAccelThreshold max ‖a − (a·ĝ_ref)ĝ_ref‖ (m/s²) w.r.t. the sensor-only gravity reference.
    * @return true if the accelerometer can be trusted as a gravity reference this tick.
    */
   public boolean isQuasiStatic(Vector3DReadOnly specificForceBody,
                                Vector3DReadOnly rawAngularVelocity,
                                double accelToleranceRatio,
                                double gyroThreshold,
                                double horizontalAccelThreshold)
   {
      double accelNorm = specificForceBody.norm();
      boolean accelNearGravity = Math.abs(accelNorm - gravityMagnitude) <= accelToleranceRatio * gravityMagnitude;
      // RAW gyro, not the bias-corrected one: a runaway upstream bias must not be able to close this gate either.
      boolean lowRotation = rawAngularVelocity.norm() <= gyroThreshold;

      // a_horiz = a − (a·ĝ_ref)ĝ_ref, with ĝ_ref the sensor-only gravity reference (NOT R̂ᵀe_z — see Javadoc).
      // Until the reference has been seeded, admit the tick on the other two terms rather than block on a zero.
      boolean lowHorizontalAccel = true;
      if (gravityReferenceInitialized)
      {
         horizontalAccel.set(gravityReferenceBody);
         horizontalAccel.scale(-specificForceBody.dot(gravityReferenceBody));
         horizontalAccel.add(specificForceBody);
         lowHorizontalAccel = horizontalAccel.norm() <= horizontalAccelThreshold;
      }

      return accelNearGravity && lowRotation && lowHorizontalAccel;
   }

   /**
    * Advances the quasi-static gate's gravity reference ĝ_ref. <b>Call once per tick, before
    * {@link #isQuasiStatic}</b>, with raw (not bias-corrected, not filter-derived) sensor values.
    *
    * <p>ĝ_ref is a complementary filter: gyro-propagate the previous reference so it stays fixed in the world
    * as the body rotates, then blend it toward the measured specific-force direction with time constant
    * {@link #GRAVITY_REFERENCE_TIME_CONSTANT}. Since the world up-axis in body coordinates evolves as
    * {@code ĝ⁺ = exp(−ω Δt)·ĝ ≈ ĝ − Δt (ω × ĝ)}, the gyro supplies the high-frequency content and the
    * accelerometer pins the low-frequency level, so the reference neither lags a rotating body nor inherits the
    * accelerometer's transient noise.</p>
    *
    * @param specificForceBody   the body-frame specific force a. Not modified.
    * @param rawAngularVelocity  the RAW body-frame gyro ω (no bias correction). Not modified.
    * @param dt                  the estimator timestep (s).
    */
   public void updateGravityReference(Vector3DReadOnly specificForceBody, Vector3DReadOnly rawAngularVelocity, double dt)
   {
      double accelNorm = specificForceBody.norm();
      if (accelNorm < 1.0e-9)
         return;

      if (!gravityReferenceInitialized)
      {
         gravityReferenceBody.setAndScale(1.0 / accelNorm, specificForceBody);
         gravityReferenceInitialized = true;
         return;
      }

      // ĝ⁻ = ĝ − Δt (ω × ĝ): carry the reference with the body using the raw gyro.
      referenceRate.cross(rawAngularVelocity, gravityReferenceBody);
      gravityReferenceBody.scaleAdd(-dt, referenceRate, gravityReferenceBody);

      // ĝ_ref = α·ĝ⁻ + (1−α)·f̂ : pin the low-frequency level to the accelerometer.
      double alpha = Math.exp(-dt / GRAVITY_REFERENCE_TIME_CONSTANT);
      gravityReferenceBody.scale(alpha);
      gravityReferenceBody.scaleAdd((1.0 - alpha) / accelNorm, specificForceBody, gravityReferenceBody);

      double norm = gravityReferenceBody.norm();
      if (norm > 1.0e-9)
         gravityReferenceBody.scale(1.0 / norm);
   }

   /** The sensor-only gravity reference ĝ_ref used by {@link #isQuasiStatic} (unit, body frame). For tests. */
   public Vector3DReadOnly getGravityReference()
   {
      return gravityReferenceBody;
   }

   public DMatrixRMaj getMeasurementJacobian()  { return measurementJacobian; }
   public DMatrixRMaj getResidual()             { return residual; }
   public DMatrixRMaj getMeasurementCovariance(){ return measurementCovariance; }

   /** Total tilt error angle (rad) between the RAW accelerometer direction and the predicted one — valid every tick. */
   public double getTiltErrorAngle()
   {
      return Math.atan2(tiltErrorVector.norm(), predictedGravityBody.dot(rawMeasuredGravityBody));
   }

   /** Body-frame roll component (rad) of the tilt error (x of ĝ_body × f̂). */
   public double getTiltErrorRoll()  { return tiltErrorVector.getX(); }

   /** Body-frame pitch component (rad) of the tilt error (y of ĝ_body × f̂). */
   public double getTiltErrorPitch() { return tiltErrorVector.getY(); }
}
