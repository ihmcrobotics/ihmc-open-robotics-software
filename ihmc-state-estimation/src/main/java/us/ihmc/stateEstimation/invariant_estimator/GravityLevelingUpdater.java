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
 * {@code (H, residual, R = σ_tilt²·I₃)} to {@link InvariantUpdater#update} performs the correction; the null
 * (yaw) direction is regularized by {@code R}, so {@code S = HPHᵀ + R} stays invertible.</p>
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

   private final DMatrixRMaj measurementJacobian;      // H (3×m)
   private final DMatrixRMaj residual = new DMatrixRMaj(MEASUREMENT_SIZE, 1);
   private final DMatrixRMaj measurementCovariance = new DMatrixRMaj(MEASUREMENT_SIZE, MEASUREMENT_SIZE); // σ_tilt² I₃

   private double tiltMeasurementVariance;             // σ_tilt² (rad², on the unit-direction residual)

   // Pre-allocated scratch (allocation-free hot path).
   private final RotationMatrix rotation = new RotationMatrix();
   private final Vector3D predictedGravityBody = new Vector3D();  // ĝ_body = R̂ᵀ e_z
   private final Vector3D measuredGravityBody = new Vector3D();   // f̂ = a/‖a‖
   private final Vector3D rTransposeEx = new Vector3D();          // R̂ᵀ e_x
   private final Vector3D rTransposeEy = new Vector3D();          // R̂ᵀ e_y
   private final Vector3D tiltErrorVector = new Vector3D();       // ĝ_body × f̂ (≈ body-frame roll/pitch error)

   private static final Vector3D E_X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D E_Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D E_Z = new Vector3D(0.0, 0.0, 1.0);

   /**
    * @param tangentSize             the tangent/covariance dimension m (= 9 + 3N).
    * @param tiltMeasurementVariance σ_tilt² — measurement noise on the unit gravity-direction residual (rad²).
    * @param gravityMagnitude        |g| (m/s²), used only by the quasi-static gate.
    */
   public GravityLevelingUpdater(int tangentSize, double tiltMeasurementVariance, double gravityMagnitude)
   {
      this.tangentSize = tangentSize;
      this.tiltMeasurementVariance = tiltMeasurementVariance;
      this.gravityMagnitude = gravityMagnitude;
      this.measurementJacobian = new DMatrixRMaj(MEASUREMENT_SIZE, tangentSize);
      setTiltMeasurementVariance(tiltMeasurementVariance);
   }

   /** Sets σ_tilt² (rad²) and rebuilds R = σ_tilt²·I₃. */
   public void setTiltMeasurementVariance(double tiltMeasurementVariance)
   {
      this.tiltMeasurementVariance = tiltMeasurementVariance;
      measurementCovariance.zero();
      for (int i = 0; i < MEASUREMENT_SIZE; i++)
         measurementCovariance.set(i, i, tiltMeasurementVariance);
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
    * (‖a‖ ≈ |g|) and the body is barely rotating (‖ω‖ small). Applying the update outside this window would
    * mistake real acceleration for tilt.
    *
    * @param specificForceBody  the bias-corrected body-frame specific force a. Not modified.
    * @param angularVelocity    the bias-corrected body-frame angular velocity ω. Not modified.
    * @param accelToleranceRatio allowed fractional deviation of ‖a‖ from |g| (e.g. 0.05 = ±5%).
    * @param gyroThreshold      max ‖ω‖ (rad/s) for "barely rotating".
    * @return true if the accelerometer can be trusted as a gravity reference this tick.
    */
   public boolean isQuasiStatic(Vector3DReadOnly specificForceBody,
                                Vector3DReadOnly angularVelocity,
                                double accelToleranceRatio,
                                double gyroThreshold)
   {
      double accelNorm = specificForceBody.norm();
      boolean accelNearGravity = Math.abs(accelNorm - gravityMagnitude) <= accelToleranceRatio * gravityMagnitude;
      boolean lowRotation = angularVelocity.norm() <= gyroThreshold;
      return accelNearGravity && lowRotation;
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
