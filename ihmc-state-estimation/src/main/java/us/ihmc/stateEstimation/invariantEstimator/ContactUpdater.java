package us.ihmc.stateEstimation.invariantEstimator;

import org.apache.commons.lang3.NotImplementedException;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Contact forward-kinematics measurement subpiece for the right-invariant EKF.
 *
 * <p>This is an optional subpiece <em>owned and called by</em> {@link InvariantUpdater}: it only
 * assembles the measurement triple (H, residual, R_world) for one contact and exposes it via getters.
 * It performs no Kalman correction itself — the main updater calls {@link #assemble} and then runs its
 * own generic correction on the assembled matrices.</p>
 *
 * <p><b>Measurement model.</b> For a contact (foot) Cᵢ, leg kinematics give the body-frame contact
 * position as a function of the joint configuration q:
 * <pre>
 *   h_Cᵢ(q) = Rᵀ·(p_Cᵢ − p_B) + J_Cᵢ(q)·w_q
 * </pre>
 * with R = {@code ᵂR_B}, p_B the base position, p_Cᵢ the contact position (all world-frame),
 * J_Cᵢ(q) the body-frame contact Jacobian and w_q the joint-encoder noise. The first term is the
 * deterministic forward kinematics; the second is the encoder-noise contribution.</p>
 *
 * <p><b>Right-invariant form.</b> The encoder reading y = h_Cᵢ(q) is a body-frame vector. Rather than
 * innovating in the body frame (state-dependent Jacobian), the innovation is rotated to the world frame:
 * <pre>
 *   residual = R̂·y − (d̂ᵢ − p̂)
 * </pre>
 * where d̂ᵢ = p̂_Cᵢ and p̂ are the estimated contact and base-position columns. Under the package error
 * convention X̂ = exp(ξ)·X with tangent ordering [δφ; δv; δp; δp_c0; …], a first-order expansion gives
 * {@code residual ≈ δp − δp_cᵢ}, so the Jacobian H is <em>constant</em>: +I on the base-position block,
 * −I on contact i's block, zeros elsewhere. The body-frame noise Nᵢ = J_Cᵢ·Σ_q·J_Cᵢᵀ is rotated in as
 * {@code R_world = R̂·Nᵢ·R̂ᵀ}.</p>
 *
 * <p><b>Learned module.</b> {@link #assemble} takes an {@code includeLearnedModule} flag. When set (and a
 * {@link LearnedContactCorrection} has been installed via {@link #setLearnedContactCorrection}), the
 * learned module is given a chance to augment the assembled (residual, H, R_world) before the caller
 * runs the correction. This is the seam for a future learned correction; with the flag off the result
 * is the plain analytic contact measurement.</p>
 *
 * <p>The assembly work matrices are pre-allocated for a single 3-DOF contact, so {@link #assemble} is
 * allocation-free.</p>
 */
public class ContactUpdater
{
   /** Innovation dimension of one contact measurement (a 3D position). */
   private static final int MEASUREMENT_SIZE = 3;

   // Pre-allocated assembly outputs, exposed via getters.
   private final DMatrixRMaj measurementJacobian;     // H        (3×m)
   private final DMatrixRMaj residual = new DMatrixRMaj(MEASUREMENT_SIZE, 1);          // r (3×1)
   private final DMatrixRMaj measurementCovariance = new DMatrixRMaj(MEASUREMENT_SIZE, MEASUREMENT_SIZE); // R_world (3×3)

   // Pre-allocated 3D temporaries.
   private final RotationMatrix rotation = new RotationMatrix();
   private final Vector3D basePosition = new Vector3D();
   private final Vector3D contactPosition = new Vector3D();
   private final Vector3D rotatedMeasurement = new Vector3D();
   private final Vector3D worldDifference = new Vector3D();
   private final Matrix3D rotatedCovariance = new Matrix3D();

   /** Optional learned correction, applied only when {@code includeLearnedModule} is set. */
   private LearnedContactCorrection learnedContactCorrection = null;

   /**
    * @param numberOfContacts the number of contact columns N (≥ 0), matching the filter state.
    */
   public ContactUpdater(int numberOfContacts)
   {
      if (numberOfContacts < 0)
         throw new IllegalArgumentException("numberOfContacts must be >= 0, was " + numberOfContacts);
      int tangentSize = 9 + 3 * numberOfContacts; // m = 9 + 3N
      measurementJacobian = new DMatrixRMaj(MEASUREMENT_SIZE, tangentSize);
   }

   /**
    * Integrates (or clears, with {@code null}) the learned correction module.
    *
    * @param learnedContactCorrection the learned correction, or {@code null} to disable it. Stored by reference.
    */
   public void setLearnedContactCorrection(LearnedContactCorrection learnedContactCorrection)
   {
      this.learnedContactCorrection = learnedContactCorrection;
   }

   /**
    * Assembles the measurement triple (H, residual, R_world) for one contact into the internal buffers,
    * optionally folding in the learned module. After this call the results are available via
    * {@link #getMeasurementJacobian()}, {@link #getResidual()} and {@link #getMeasurementCovariance()}.
    *
    * @param state                     the estimate state (read). Not modified.
    * @param contactIndex              the contact index i in [0, N).
    * @param bodyMeasurement           the body-frame forward-kinematics measurement y = h_Cᵢ(q). Not modified.
    * @param bodyMeasurementCovariance the body-frame measurement covariance Nᵢ = J_Cᵢ·Σ_q·J_Cᵢᵀ (3×3). Not modified.
    * @param includeLearnedModule      if {@code true} and a {@link LearnedContactCorrection} is installed,
    *                                  the learned module augments (residual, H, R_world) in place.
    */
   public void assemble(InvariantState state,
                        int contactIndex,
                        Tuple3DReadOnly bodyMeasurement,
                        Matrix3DReadOnly bodyMeasurementCovariance,
                        boolean includeLearnedModule)
   {
      if (includeLearnedModule == true)
         throw new NotImplementedException("Learned module not implemented yet, please pass false here.");
      computeResidual(state, contactIndex, bodyMeasurement, residual);
      computeJacobian(state, contactIndex, measurementJacobian);
      computeMeasurementCovariance(state, bodyMeasurementCovariance, measurementCovariance);
   }

   /** @return the most recently assembled measurement Jacobian H (3×m). Live buffer; do not mutate externally. */
   public DMatrixRMaj getMeasurementJacobian()
   {
      return measurementJacobian;
   }

   /** @return the most recently assembled world-frame residual r (3×1). Live buffer; do not mutate externally. */
   public DMatrixRMaj getResidual()
   {
      return residual;
   }

   /** @return the most recently assembled world-frame measurement covariance R (3×3). Live buffer; do not mutate externally. */
   public DMatrixRMaj getMeasurementCovariance()
   {
      return measurementCovariance;
   }

   /**
    * Computes the world-frame right-invariant residual r = R̂·y − (d̂ᵢ − p̂).
    *
    * @param state           the estimate state (read). Not modified.
    * @param contactIndex    the contact index i in [0, N).
    * @param bodyMeasurement the body-frame measurement y = h_Cᵢ(q). Not modified.
    * @param residualToPack  the 3×1 residual to fill; reshaped. Modified.
    */
   public void computeResidual(InvariantState state, int contactIndex, Tuple3DReadOnly bodyMeasurement, DMatrixRMaj residualToPack)
   {
      state.getRotation(rotation);
      state.getBasePosition(basePosition);
      state.getContactPosition(contactIndex, contactPosition);

      rotation.transform(bodyMeasurement, rotatedMeasurement);
      worldDifference.sub(contactPosition, basePosition);

      residualToPack.reshape(MEASUREMENT_SIZE, 1);
      residualToPack.set(0, 0, rotatedMeasurement.getX() - worldDifference.getX());
      residualToPack.set(1, 0, rotatedMeasurement.getY() - worldDifference.getY());
      residualToPack.set(2, 0, rotatedMeasurement.getZ() - worldDifference.getZ());
   }

   /**
    * Builds the state-independent measurement Jacobian H (3×m): +I on the base-position block,
    * −I on contact i's block, zeros elsewhere.
    *
    * @param state          used only for the tangent-block offsets and size; values not read.
    * @param contactIndex   the contact index i in [0, N).
    * @param jacobianToPack the 3×m Jacobian to fill; reshaped and zeroed. Modified.
    */
   public void computeJacobian(InvariantState state, int contactIndex, DMatrixRMaj jacobianToPack)
   {
      jacobianToPack.reshape(MEASUREMENT_SIZE, state.getTangentSize());
      jacobianToPack.zero();

      setScaledIdentityBlock(jacobianToPack, state.basePositionTangentIndex(), +1.0);
      setScaledIdentityBlock(jacobianToPack, state.contactTangentIndex(contactIndex), -1.0);

   }

   /**
    * Rotates the body-frame measurement covariance into the world frame: R_world = R̂·Nᵢ·R̂ᵀ.
    *
    * @param state            the estimate state (read for R̂). Not modified.
    * @param bodyCovariance   the body-frame covariance Nᵢ = J_Cᵢ·Σ_q·J_Cᵢᵀ (3×3). Not modified.
    * @param covarianceToPack the 3×3 world-frame covariance to fill; reshaped. Modified.
    */
   public void computeMeasurementCovariance(InvariantState state, Matrix3DReadOnly bodyCovariance, DMatrixRMaj covarianceToPack)
   {
      state.getRotation(rotation);

      rotatedCovariance.set(bodyCovariance);
      rotation.transform(rotatedCovariance);

      covarianceToPack.reshape(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
      for (int r = 0; r < 3; r++)
         for (int c = 0; c < 3; c++)
            covarianceToPack.set(r, c, rotatedCovariance.getElement(r,c));
   }

   /**
    * Maps joint-encoder noise into a body-frame contact-position covariance: Nᵢ = J_Cᵢ·Σ_q·J_Cᵢᵀ,
    * realizing the {@code J_Cᵢ(q)·w_q} term of the measurement model as a covariance.
    *
    * @param contactJacobian      the body-frame contact Jacobian J_Cᵢ (3×nJoints). Not modified.
    * @param jointCovariance      the joint-encoder covariance Σ_q (nJoints×nJoints). Not modified.
    * @param bodyCovarianceToPack the 3×3 body-frame covariance Nᵢ to fill. Modified.
    */
   public static void mapEncoderNoise(DMatrixRMaj contactJacobian, DMatrixRMaj jointCovariance, Matrix3D bodyCovarianceToPack)
   {
      int nJoints = contactJacobian.getNumCols();
      if (contactJacobian.getNumRows() != MEASUREMENT_SIZE)
         throw new IllegalArgumentException("contactJacobian must be 3xnJoints, was " + contactJacobian.getNumRows() + "x" + nJoints);
      if (jointCovariance.getNumRows() != nJoints || jointCovariance.getNumCols() != nJoints)
         throw new IllegalArgumentException("jointCovariance must be " + nJoints + "x" + nJoints + ", was " + jointCovariance.getNumRows() + "x" + jointCovariance.getNumCols());

      DMatrixRMaj jSigma = new DMatrixRMaj(MEASUREMENT_SIZE, nJoints);
      DMatrixRMaj nBody = new DMatrixRMaj(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
      CommonOps_DDRM.mult(contactJacobian, jointCovariance, jSigma);
      CommonOps_DDRM.multTransB(jSigma, contactJacobian, nBody);

      bodyCovarianceToPack.set(nBody.get(0,0), nBody.get(0,1), nBody.get(0,2),
                               nBody.get(1,0), nBody.get(1,1), nBody.get(1,2),
                               nBody.get(2,0),nBody.get(2,1),nBody.get(2,2));

   }

   /** Writes scale·I into the 3×3 block of {@code dest} whose first column/row is at {@code blockStart}. */
   private static void setScaledIdentityBlock(DMatrixRMaj dest, int blockStart, double scale)
   {
      for (int j = 0; j < 3; j++)
         dest.set(j, blockStart + j, scale);
   }

   /**
    * Seam for a future learned contact correction. Implementations may adjust the assembled measurement
    * (residual, Jacobian, and/or noise covariance) in place before the analytic correction is applied;
    * it is invoked only when {@code assemble(..., includeLearnedModule=true)} and an instance is installed.
    */
   public interface LearnedContactCorrection
   {
      /**
       * @param state                 the current estimate (read-only use expected).
       * @param contactIndex          the contact index i in [0, N).
       * @param residual              the assembled 3×1 residual; may be modified in place.
       * @param measurementJacobian   the assembled 3×m Jacobian H; may be modified in place.
       * @param measurementCovariance the assembled 3×3 world-frame covariance R; may be modified in place.
       */
      void augment(InvariantState state,
                   int contactIndex,
                   DMatrixRMaj residual,
                   DMatrixRMaj measurementJacobian,
                   DMatrixRMaj measurementCovariance);
   }
}
