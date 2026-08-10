package us.ihmc.stateEstimation.invariantEstimator;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.lieGroup.SO3LieGroupTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Discrete-time prediction (propagation) step for the world-centric right-invariant EKF on SE_k(3).
 *
 * <p>Inputs are the bias-corrected body-frame angular velocity ω and specific force a (an upstream
 * joint-space EKF owns the biases), gravity g, and the timestep Δt. The mean uses the exact discrete
 * update with the SO(3) integration coefficients Γ₀/Γ₁/Γ₂; the covariance uses
 * {@code P⁺ = Φ P Φᵀ + Q_d} with the state-independent transition Φ and the group adjoint mapping
 * the process noise.</p>
 *
 * <p>Mean update (φ = ωΔt):
 * <pre>
 *   R⁺ = R · exp(φ)
 *   v⁺ = v + g·Δt + R · Γ₁(φ) · a · Δt
 *   p⁺ = p + v·Δt + ½·g·Δt² + R · Γ₂(φ) · a · Δt²
 *   dᵢ⁺ = dᵢ
 * </pre>
 * Covariance: Φ = exp(A_c Δt) (depends only on g and Δt); Q_d = Φ · Ad_X · Q_c · Ad_Xᵀ · Φᵀ · Δt.</p>
 *
 * <p>The covariance is linearized about the prior state, so {@link #predict} propagates P
 * <em>before</em> overwriting X with the mean update.</p>
 */
public class InvariantPropagator
{
   private final int numberOfContacts;
   /** World-frame gravity g = (0, 0, -|g|). Set from the robot process's gravity, never hard-coded here. */
   private final Vector3D gravity = new Vector3D();

   /** Continuous process-noise covariance Q_c (m×m); the contact-slip blocks may be retuned per tick. */
   private final DMatrixRMaj processNoise;

   /** Tangent-space start index of contact i's 3×3 block in Q_c (and P): 9 + 3i. */
   private static final int FIRST_CONTACT_TANGENT_INDEX = 9;

   // Pre-allocated work matrices (m×m) to keep predict() allocation-free.
   private final DMatrixRMaj Phi;
   private final DMatrixRMaj adjoint;
   private final DMatrixRMaj processNoiseDiscrete;
   private final DMatrixRMaj tempA;
   private final DMatrixRMaj tempB;

   // Pre-allocated 3D temporaries for the mean update.
   private final Vector3D phi = new Vector3D();
   private final RotationMatrix rotation = new RotationMatrix();
   private final RotationMatrix rotationIncrement = new RotationMatrix();
   private final RotationMatrix newRotation = new RotationMatrix();
   private final Vector3D velocity = new Vector3D();
   private final Vector3D position = new Vector3D();
   private final Vector3D newVelocity = new Vector3D();
   private final Vector3D newPosition = new Vector3D();
   private final Vector3D term = new Vector3D();
   private final Matrix3D gammaMatrix = new Matrix3D();
   private final Matrix3D hatGravity = new Matrix3D();

   // Pre-allocated scratch for the allocation-free SEK3_Utils.adjoint call in predictCovariance().
   private final RotationMatrix adjointRotation = new RotationMatrix();
   private final Vector3D adjointColumn = new Vector3D();
   private final Matrix3D adjointHat = new Matrix3D();

   /**
    * @param numberOfContacts the number of contact columns N (≥ 0).
    * @param gyroVariance     continuous angular-velocity noise variance σ_ω² (rad²/s).
    * @param accelVariance    continuous specific-force noise variance σ_a² (m²/s³).
    * @param contactVariance  continuous contact-slip noise variance σ_c² (m²/s).
    * @param gravitationalAcceleration the process gravity (m/s²); sign not considered, g is always (0, 0, −|g|).
    */
   public InvariantPropagator(int numberOfContacts, double gyroVariance, double accelVariance, double contactVariance, double gravitationalAcceleration)
   {
      if (numberOfContacts < 0)
         throw new IllegalArgumentException("numberOfContacts must be >= 0, was " + numberOfContacts);

      this.numberOfContacts = numberOfContacts;
      gravity.set(0.0, 0.0, -Math.abs(gravitationalAcceleration));

      int m = 3 + 3 * (2 + numberOfContacts); // tangent size = 9 + 3N

      processNoise = new DMatrixRMaj(m, m);
      Phi = new DMatrixRMaj(m, m);
      adjoint = new DMatrixRMaj(m, m);
      processNoiseDiscrete = new DMatrixRMaj(m, m);
      tempA = new DMatrixRMaj(m, m);
      tempB = new DMatrixRMaj(m, m);

      buildProcessNoise(gyroVariance, accelVariance, contactVariance);
   }

   /**
    * Retunes contact i's continuous slip-noise variance σ_{c,i}² in Q_c, in place. The soft contact
    * handling calls this each tick to inflate a swing foot's anchor noise (so it re-anchors softly on
    * touchdown) and restore it in stance. Isotropic: writes {@code variance·I} on contact i's 3×3 block.
    *
    * @param contactIndex the contact index i in [0, N).
    * @param variance     the continuous slip variance σ_{c,i}² (m²/s), ≥ 0.
    */
   public void setContactSlipVariance(int contactIndex, double variance)
   {
      if (contactIndex < 0 || contactIndex >= numberOfContacts)
         throw new IndexOutOfBoundsException("contactIndex must be in [0, " + numberOfContacts + "), was " + contactIndex);
      if (variance < 0.0)
         throw new IllegalArgumentException("variance must be >= 0, was " + variance);

      int block = FIRST_CONTACT_TANGENT_INDEX + 3 * contactIndex;
      for (int j = 0; j < 3; j++)
         processNoise.set(block + j, block + j, variance);
   }

   /**
    * Propagates the state forward by Δt. Covariance is propagated first (about the prior X),
    * then the mean.
    *
    * @param state             the state to update in place. Modified.
    * @param angularVelocity   bias-corrected body-frame ω. Not modified.
    * @param linearAcceleration bias-corrected body-frame specific force a. Not modified.
    * @param dt                the timestep Δt (> 0).
    */
   public void predict(InvariantState state, Vector3DReadOnly angularVelocity, Vector3DReadOnly linearAcceleration, double dt)
   {
      predictCovariance(state, dt);
      predictMean(state, angularVelocity, linearAcceleration, dt);
   }

   /** Mean update R⁺/v⁺/p⁺ using Γ₀/Γ₁/Γ₂; contacts unchanged. */
   private void predictMean(InvariantState state, Vector3DReadOnly angularVelocity, Vector3DReadOnly linearAcceleration, double dt)
   {
      phi.setAndScale(dt, angularVelocity);

      state.getRotation(rotation);
      state.getBaseVelocity(velocity);
      state.getBasePosition(position);

      // p⁺ = p + v·Δt + ½·g·Δt² + R·Γ₂(φ)·a·Δt²   (uses prior v, so compute before v is overwritten)
      SO3LieGroupTools.gamma2(phi, gammaMatrix);
      gammaMatrix.transform(linearAcceleration, term); // term = Γ₂·a
      rotation.transform(term);                        // term = R·Γ₂·a
      newPosition.set(position);
      newPosition.scaleAdd(dt, velocity, newPosition);           // + v·Δt
      newPosition.scaleAdd(0.5 * dt * dt, gravity, newPosition); // + ½·g·Δt²
      newPosition.scaleAdd(dt * dt, term, newPosition);          // + R·Γ₂·a·Δt²

      // v⁺ = v + g·Δt + R·Γ₁(φ)·a·Δt
      SO3LieGroupTools.leftJacobian(phi, gammaMatrix);
      gammaMatrix.transform(linearAcceleration, term); // term = Γ₁·a
      rotation.transform(term);                        // term = R·Γ₁·a
      newVelocity.set(velocity);
      newVelocity.scaleAdd(dt, gravity, newVelocity);  // + g·Δt
      newVelocity.scaleAdd(dt, term, newVelocity);     // + R·Γ₁·a·Δt

      // R⁺ = R·exp(φ)
      SO3LieGroupTools.exp(phi, rotationIncrement);
      newRotation.set(rotation);
      newRotation.multiply(rotationIncrement);

      state.setRotation(newRotation);
      state.setBaseVelocity(newVelocity);
      state.setBasePosition(newPosition);
      // contact columns are static in the world frame, propagation model is to simply keep them the same.

   }

   /** Covariance update P⁺ = Φ P Φᵀ + Q_d, linearized about the prior X. */
   private void predictCovariance(InvariantState state, double dt)
   {
      buildStateTransition(dt);
      SEK3Utils.adjoint(state.getGroupElement(), adjoint, adjointRotation, adjointColumn, adjointHat);

      // Q_d = Φ · Ad · Q_c · Adᵀ · Φᵀ · Δt
      CommonOps_DDRM.mult(adjoint, processNoise, tempA);           // Ad·Q_c
      CommonOps_DDRM.multTransB(tempA, adjoint, tempB);            // Ad·Q_c·Adᵀ
      CommonOps_DDRM.mult(Phi, tempB, tempA);                     // Φ·(Ad·Q_c·Adᵀ)
      CommonOps_DDRM.multTransB(tempA, Phi, processNoiseDiscrete); // Φ·Ad·Q_c·Adᵀ·Φᵀ
      CommonOps_DDRM.scale(dt, processNoiseDiscrete);             // × Δt

      // P⁺ = Φ·P·Φᵀ + Q_d
      DMatrixRMaj covariance = state.getCovariance();
      CommonOps_DDRM.mult(Phi, covariance, tempA);  // Φ·P
      CommonOps_DDRM.multTransB(tempA, Phi, tempB); // Φ·P·Φᵀ
      CommonOps_DDRM.add(tempB, processNoiseDiscrete, covariance); // write back into P

   }

   /** Builds the state-independent transition Φ = exp(A_c Δt) into {@link #Phi}. */
   private void buildStateTransition(double dt)
   {
      CommonOps_DDRM.setIdentity(Phi);
      SO3LieGroupTools.hat(gravity, hatGravity);

      setScaledBlock(Phi, 3, 0, hatGravity, dt);
      setScaledBlock(Phi, 6, 0, hatGravity, 0.5 * dt * dt);
      addScaledIdentity(Phi, 6, 3, dt);
   }

   /** Builds the continuous process-noise covariance Q_c into {@link #processNoise}. */
   private void buildProcessNoise(double gyroVariance, double accelVariance, double contactVariance)
   {
      processNoise.zero();
      addScaledIdentity(processNoise, 0,0, gyroVariance);
      addScaledIdentity(processNoise, 3,3, accelVariance);
      // p block (6,6) carries no direct noise
      for (int i = 0; i < numberOfContacts; i++)
         addScaledIdentity(processNoise, 9 + 3 * i, 9 + 3 * i, contactVariance);

   }

   /** Adds scale·I into the 3×3 block of {@code dest} at (row0, col0). */
   private static void addScaledIdentity(DMatrixRMaj dest, int row0, int col0, double scale)
   {
      for (int j = 0; j < 3; j++)
         dest.add(row0 + j, col0 + j, scale);
   }

   /** Writes scale·block into the 3×3 block of {@code dest} at (row0, col0). */
   private static void setScaledBlock(DMatrixRMaj dest, int row0, int col0, Matrix3DReadOnly block, double scale)
   {
      for (int r = 0; r < 3; r++)
         for (int c = 0; c < 3; c++)
            dest.set(row0 + r, col0 + c, scale * block.getElement(r,c));
   }
}
