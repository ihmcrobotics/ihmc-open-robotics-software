package us.ihmc.stateEstimation.invariant_estimator;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Generic right-invariant EKF correction (measurement update) core.
 *
 * <p>Given a linearized measurement {@code residual ≈ H·ξ} — where ξ is the tangent error with the
 * convention {@code X̂ = exp(ξ)·X} used throughout this package — and the measurement-noise covariance
 * R (in the same innovation space as the residual), this performs the Kalman correction:
 * <pre>
 *   S = H·P·Hᵀ + R
 *   K = P·Hᵀ·S⁻¹
 *   c = K·residual
 *   X̂⁺ = exp_G(−c)·X̂          (left-multiplicative invariant update)
 *   P⁺ = (I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ   (Joseph form)
 * </pre>
 * </p>
 *
 * <p>The measurement dimension (H's row count) may vary between calls — e.g. when the number of
 * active contacts changes — so the measurement-sized work matrices are reshaped per call; this class
 * is therefore not strictly allocation-free under a changing measurement size.</p>
 */
public class InvariantUpdater
{
   private final double[] correctionArray;

   // Fixed-size (m × m or n × n) work; measurement-sized work is reshaped per call.
   private final DMatrixRMaj innovationCovariance = new DMatrixRMaj(1, 1);    // S   (z×z)
   private final DMatrixRMaj innovationCovarianceInverse = new DMatrixRMaj(1, 1); // S⁻¹
   private final DMatrixRMaj hTimesCovariance = new DMatrixRMaj(1, 1);        // H·P (z×m)
   private final DMatrixRMaj covarianceTimesHTranspose = new DMatrixRMaj(1, 1); // P·Hᵀ (m×z)
   private final DMatrixRMaj gain = new DMatrixRMaj(1, 1);                    // K   (m×z)
   private final DMatrixRMaj gainTimesNoise = new DMatrixRMaj(1, 1);          // K·R (m×z)
   private final DMatrixRMaj correctionColumn = new DMatrixRMaj(1, 1);        // c   (m×1)
   private final DMatrixRMaj gainTimesH;                                      // K·H (m×m)
   private final DMatrixRMaj identityMinusKH;                                 // I−KH (m×m)
   private final DMatrixRMaj covarianceWork;                                  // (m×m)
   private final DMatrixRMaj expDelta = new DMatrixRMaj(1, 1);                // exp_G(−c) (n×n)
   private final DMatrixRMaj newGroupElement = new DMatrixRMaj(1, 1);         // (n×n)

   /** Optional contact measurement subpiece, owned and called by this updater (see {@link #updateContact}). */
   private ContactUpdater contactUpdater = null;

   /**
    * @param tangentSize the tangent/covariance dimension m (= 9 + 3N).
    */
   public InvariantUpdater(int tangentSize)
   {
      correctionArray = new double[tangentSize];
      gainTimesH = new DMatrixRMaj(tangentSize, tangentSize);
      identityMinusKH = new DMatrixRMaj(tangentSize, tangentSize);
      covarianceWork = new DMatrixRMaj(tangentSize, tangentSize);
   }

   /**
    * Integrates (or clears, with {@code null}) the contact measurement subpiece used by {@link #updateContact}.
    *
    * @param contactUpdater the contact measurement assembler, or {@code null} to disable contact updates.
    *                       Stored by reference; its contact count must match this updater's tangent size.
    */
   public void setContactUpdater(ContactUpdater contactUpdater)
   {
      this.contactUpdater = contactUpdater;
   }

   /**
    * Main contact correction entry point: assembles the contact forward-kinematics measurement through
    * the owned {@link ContactUpdater} subpiece (via {@link #updateContact}) and then applies the generic
    * right-invariant correction to the state in place.
    *
    * <p>This is the orchestrator — it calls {@link #updateContact} to build (H, residual, R_world) and
    * then the core {@link #update(InvariantState, DMatrixRMaj, DMatrixRMaj, DMatrixRMaj)} to correct.</p>
    *
    * @param state                     the state whose X and P are updated in place. Modified.
    * @param contactIndex              the contact index i in [0, N).
    * @param bodyMeasurement           the body-frame forward-kinematics measurement y = h_Cᵢ(q). Not modified.
    * @param bodyMeasurementCovariance the body-frame measurement covariance Nᵢ = J_Cᵢ·Σ_q·J_Cᵢᵀ (3×3). Not modified.
    * @param includeLearnedModule      forwarded to the subpiece: if {@code true} and a learned correction
    *                                  is installed there, it augments the assembled measurement.
    * @throws IllegalStateException if no {@link ContactUpdater} has been installed.
    */
   public void update(InvariantState state,
                      int contactIndex,
                      Vector3DReadOnly bodyMeasurement,
                      Matrix3DReadOnly bodyMeasurementCovariance,
                      boolean includeLearnedModule)
   {
      updateContact(state, contactIndex, bodyMeasurement, bodyMeasurementCovariance, includeLearnedModule);

      update(state, contactUpdater.getMeasurementJacobian(), contactUpdater.getResidual(), contactUpdater.getMeasurementCovariance());
   }

   /**
    * Applies one right-invariant correction to the state in place.
    *
    * @param state                 the state whose X and P are updated in place. Modified.
    * @param H                     the measurement Jacobian (z×m), with {@code residual ≈ H·ξ}. Not modified.
    * @param residual              the innovation (z×1). Not modified.
    * @param measurementCovariance R, the innovation-space noise covariance (z×z). Not modified.
    */
   public void update(InvariantState state, DMatrixRMaj H, DMatrixRMaj residual, DMatrixRMaj measurementCovariance)
   {
      DMatrixRMaj covariance = state.getCovariance();
      int m = covariance.getNumRows();
      int z  = H.getNumRows();

      // S = H·P·Hᵀ + R
      hTimesCovariance.reshape(z,m);
      CommonOps_DDRM.mult(H, covariance, hTimesCovariance);
      innovationCovariance.reshape(z,z);
      CommonOps_DDRM.multTransB(hTimesCovariance, H, innovationCovariance);
      CommonOps_DDRM.addEquals(innovationCovariance, measurementCovariance);

      // K = P·Hᵀ·S⁻¹
      covarianceTimesHTranspose.reshape(m,z);
      CommonOps_DDRM.multTransB(covariance, H, covarianceTimesHTranspose);
      innovationCovarianceInverse.reshape(z,z);
      CommonOps_DDRM.invert(innovationCovariance,innovationCovarianceInverse);
      gain.reshape(m,z);
      CommonOps_DDRM.mult(covarianceTimesHTranspose,innovationCovarianceInverse,gain);

      // c = K·residual ; state update X̂⁺ = exp_G(−c)·X̂
      correctionColumn.reshape(m,1);
      CommonOps_DDRM.mult(gain, residual, correctionColumn);
      for (int i = 0; i < m; i++)
         correctionArray[i] = -correctionColumn.get(i, 0); // negative: error convention X̂ = exp(ξ)·X

      SEK3_Utils.exp(correctionArray, expDelta);
      newGroupElement.reshape(expDelta.getNumRows(), expDelta.getNumCols());
      CommonOps_DDRM.mult(expDelta, state.getGroupElement(), newGroupElement);
      state.getGroupElement().set(newGroupElement);

      // P⁺ = (I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ
      CommonOps_DDRM.mult(gain, H, gainTimesH); // KH
      CommonOps_DDRM.setIdentity(identityMinusKH);
      CommonOps_DDRM.subtractEquals(identityMinusKH, gainTimesH); // I - KH


      CommonOps_DDRM.mult(identityMinusKH, covariance, covarianceWork); // (I-KH)P
      CommonOps_DDRM.multTransB(covarianceWork, identityMinusKH, covariance); // (I-KH)P(I-KH)^T

      gainTimesNoise.reshape(m,z);
      CommonOps_DDRM.mult(gain, measurementCovariance, gainTimesNoise); // K * R
      CommonOps_DDRM.multAddTransB(gainTimesNoise, gain, covariance); // P += KRK^T

   }

   /**
    * Assembles the contact forward-kinematics measurement through the owned {@link ContactUpdater}
    * subpiece, leaving (H, residual, R_world) available on it. Does <em>not</em> apply the correction —
    * the orchestrating {@link #update(InvariantState, int, Vector3DReadOnly, Matrix3DReadOnly, boolean)}
    * runs the core correction afterwards.
    *
    * @param state                     the estimate state (read). Not modified.
    * @param contactIndex              the contact index i in [0, N).
    * @param bodyMeasurement           the body-frame forward-kinematics measurement y = h_Cᵢ(q). Not modified.
    * @param bodyMeasurementCovariance the body-frame measurement covariance Nᵢ = J_Cᵢ·Σ_q·J_Cᵢᵀ (3×3). Not modified.
    * @param includeLearnedModule      forwarded to the subpiece: if {@code true} and a learned correction
    *                                  is installed there, it augments the assembled measurement.
    * @throws IllegalStateException if no {@link ContactUpdater} has been installed.
    */
   private void updateContact(InvariantState state,
                              int contactIndex,
                              Vector3DReadOnly bodyMeasurement,
                              Matrix3DReadOnly bodyMeasurementCovariance,
                              boolean includeLearnedModule)
   {
      if (contactUpdater == null)
         throw new IllegalStateException("No ContactUpdater added; call setContactUpdater(...) first.");

      contactUpdater.assemble(state, contactIndex, bodyMeasurement, bodyMeasurementCovariance, includeLearnedModule);
   }
}
