package us.ihmc.stateEstimation.invariant_estimator;

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
 * <p><b>Measurement model.</b> The body-frame specific force is {@code a = Rᵀ(a_world − g)}; when quasi-static
 * {@code a_world ≈ 0}, so the normalized specific force {@code f̂ = a/‖a‖ ≈ Rᵀ e_z} is the world up-axis seen
 * in body frame. The prediction is {@code ĝ_body = R̂ᵀ e_z}. With this package's right-invariant error
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
 * roll and distrust pitch. In the measurement's body frame the pitch-informing direction is {@code û_p = R̂ᵀe_x}
 * (= H's {@code δφ_y} column) and the roll-informing direction is {@code R̂ᵀe_y}, so
 * <pre>
 *   R = σ_roll² I₃ + (σ_pitch² − σ_roll²) û_p û_pᵀ  =  diag(σ_pitch², σ_roll², σ_roll²) in {û_p, R̂ᵀe_y, ĝ}
 * </pre>
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

   // Pre-allocated scratch (allocation-free hot path).
   private final RotationMatrix rotation = new RotationMatrix();
   private final Vector3D predictedGravityBody = new Vector3D();  // ĝ_body = R̂ᵀ e_z
   private final Vector3D measuredGravityBody = new Vector3D();   // f̂ = a/‖a‖
   private final Vector3D rTransposeEx = new Vector3D();          // R̂ᵀ e_x (= û_p, the pitch-informing direction)
   private final Vector3D rTransposeEy = new Vector3D();          // R̂ᵀ e_y
   private final Vector3D tiltErrorVector = new Vector3D();       // ĝ_body × f̂ (≈ body-frame roll/pitch error)
   private final Vector3D horizontalAccel = new Vector3D();       // a − (a·ĝ)ĝ, for the quasi-static gate

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

      // ĝ_body = R̂ᵀ e_z (world up seen in body); measured f̂ = a/‖a‖.
      rotation.inverseTransform(E_Z, predictedGravityBody);
      measuredGravityBody.set(specificForceBody);
      double norm = measuredGravityBody.norm();
      if (norm > 1.0e-9)
         measuredGravityBody.scale(1.0 / norm);

      // residual = f̂ − ĝ_body ≈ −R̂ᵀ[e_z]ₓ δφ.
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

      // Anisotropic R = σ_roll² I₃ + (σ_pitch²_eff − σ_roll²) û_p û_pᵀ,  û_p = R̂ᵀe_x (the pitch-informing
      // direction). Rebuilt every tick because û_p depends on R̂. See the class Javadoc for why pitch is distrusted.
      double effectivePitchVariance = pitchObservable ? pitchMeasurementVariance : PITCH_DISABLED_VARIANCE;
      double pitchMinusRoll = effectivePitchVariance - rollMeasurementVariance;
      for (int r = 0; r < 3; r++)
      {
         double ur = elementOf(rTransposeEx, r);
         for (int c = 0; c < 3; c++)
            measurementCovariance.set(r, c, (r == c ? rollMeasurementVariance : 0.0) + pitchMinusRoll * ur * elementOf(rTransposeEx, c));
      }

      // Diagnostic: tilt-error vector ĝ_body × f̂ (small-angle rotation aligning predicted → measured).
      // Its body-frame x/y ≈ roll/pitch error; angle = atan2(‖ĝ×f̂‖, ĝ·f̂).
      tiltErrorVector.cross(predictedGravityBody, measuredGravityBody);
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
    * forward slips through. Requires {@link #assemble} to have run this tick (it supplies ĝ_body = R̂ᵀe_z).
    *
    * @param specificForceBody       the bias-corrected body-frame specific force a. Not modified.
    * @param angularVelocity         the bias-corrected body-frame angular velocity ω. Not modified.
    * @param accelToleranceRatio     allowed fractional deviation of ‖a‖ from |g| (e.g. 0.05 = ±5%).
    * @param gyroThreshold           max ‖ω‖ (rad/s) for "barely rotating".
    * @param horizontalAccelThreshold max ‖a − (a·ĝ)ĝ‖ (m/s²): horizontal specific force w.r.t. the estimated up.
    * @return true if the accelerometer can be trusted as a gravity reference this tick.
    */
   public boolean isQuasiStatic(Vector3DReadOnly specificForceBody,
                                Vector3DReadOnly angularVelocity,
                                double accelToleranceRatio,
                                double gyroThreshold,
                                double horizontalAccelThreshold)
   {
      double accelNorm = specificForceBody.norm();
      boolean accelNearGravity = Math.abs(accelNorm - gravityMagnitude) <= accelToleranceRatio * gravityMagnitude;
      boolean lowRotation = angularVelocity.norm() <= gyroThreshold;

      // a_horiz = a − (a·ĝ)ĝ, with ĝ = predictedGravityBody (unit) from the last assemble().
      horizontalAccel.set(predictedGravityBody);
      horizontalAccel.scale(-specificForceBody.dot(predictedGravityBody));
      horizontalAccel.add(specificForceBody);
      boolean lowHorizontalAccel = horizontalAccel.norm() <= horizontalAccelThreshold;

      return accelNearGravity && lowRotation && lowHorizontalAccel;
   }

   public DMatrixRMaj getMeasurementJacobian()  { return measurementJacobian; }
   public DMatrixRMaj getResidual()             { return residual; }
   public DMatrixRMaj getMeasurementCovariance(){ return measurementCovariance; }

   /** Total tilt error angle (rad) between measured and predicted gravity direction — valid every tick. */
   public double getTiltErrorAngle()
   {
      return Math.atan2(tiltErrorVector.norm(), predictedGravityBody.dot(measuredGravityBody));
   }

   /** Body-frame roll component (rad) of the tilt error (x of ĝ_body × f̂). */
   public double getTiltErrorRoll()  { return tiltErrorVector.getX(); }

   /** Body-frame pitch component (rad) of the tilt error (y of ĝ_body × f̂). */
   public double getTiltErrorPitch() { return tiltErrorVector.getY(); }
}
