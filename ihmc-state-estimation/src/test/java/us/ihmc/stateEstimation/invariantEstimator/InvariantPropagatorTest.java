package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.lieGroup.SO3LieGroupTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Tests for {@link InvariantPropagator}. Mean behaviour is checked against closed-form kinematics
 * (free fall, gravity-compensated rest, rotation composition, static contacts); covariance behaviour
 * is checked structurally (symmetry, growth from zero, zero-noise invariance).
 */
public class InvariantPropagatorTest
{
   private static final double EPSILON = 1.0e-10;
   private static final double GRAVITY = -9.81; // injected into InvariantPropagator below; the expected values assume it

   /** ω = 0, a = 0: pure free fall. After T seconds, v_z = g·T and p_z = ½·g·T² exactly, R = I. */
   @Test
   public void testFreeFall()
   {
      double dt = 1.0e-3;
      int steps = 1000;

      InvariantPropagator propagator = new InvariantPropagator(0, 0.0, 0.0, 0.0, GRAVITY); // no contacts and no noise
      InvariantState state = new InvariantState(0);

      Vector3D omega = new Vector3D(0.0, 0.0, 0.0);
      Vector3D accel = new Vector3D(0.0, 0.0, 0.0); // free fall : no specific force

      for (int i = 0; i < steps; i++)
         propagator.predict(state, omega, accel, dt);

      double time = steps * dt;
      Vector3D velocity = new Vector3D();
      Vector3D positionVector = new Vector3D();
      state.getBaseVelocity(velocity);
      state.getBasePosition(positionVector);

      // Constant accel integrator is exact, so this should hold up to numerical precision checks.
      assertEquals(0.0, velocity.getX(), EPSILON);
      assertEquals(0.0, velocity.getY(), EPSILON);
      assertEquals(GRAVITY * time, velocity.getZ(), 1.0e-9);
      assertEquals(0.5 * GRAVITY * time * time, positionVector.getZ(), 1.0e-9);

      RotationMatrix rotation = new RotationMatrix();
      state.getRotation(rotation);
      EuclidCoreTestTools.assertMatrix3DEquals(new RotationMatrix(), rotation, EPSILON); // should be the identity

   }

   /** ω = 0, a = -g (rest reaction with R = I): the base stays at the origin with zero velocity. */
   @Test
   public void testStationaryWithGravityCompensation()
   {
      double dt = 1.0e-3;
      int steps = 1000;
      InvariantPropagator propagator = new InvariantPropagator(0, 0.0, 0.0, 0.0, GRAVITY);
      InvariantState state = new InvariantState(0);

      Vector3D omega = new Vector3D(0.0, 0.0, 0.0);
      Vector3D accel = new Vector3D(0.0, 0.0, -GRAVITY); // a = -g = (0, 0, +9.81)

      for (int i = 0; i < steps; i++)
         propagator.predict(state ,omega, accel, dt);

      Vector3D velocity = new Vector3D();
      Vector3D positionVector = new Vector3D();
      state.getBaseVelocity(velocity);
      state.getBasePosition(positionVector);

      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), velocity, 1.0e-9);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), positionVector, 1.0e-9);
   }

   /** Constant ω, a = 0: after N steps R = exp(ω·T), since equal-axis increments compose. */
   @Test
   public void testConstantAngularVelocityComposesRotation()
   {
      double dt = 1.0e-3;
      int steps = 500;
      InvariantPropagator propagator = new InvariantPropagator(0, 0.0, 0.0, 0.0, GRAVITY);
      InvariantState state = new InvariantState(0);

      Vector3D omega = new Vector3D(0.3, -0.2, 0.5);
      Vector3D accel = new Vector3D(0.0, 0.0, 0.0);

      for (int i = 0; i < steps; i++)
         propagator.predict(state, omega, accel, dt);

      double time = steps * dt;
      Vector3D rotationVector = new Vector3D();
      rotationVector.setAndScale(time, omega); // \omega * T

      RotationMatrix expected = new RotationMatrix();
      SO3LieGroupTools.exp(rotationVector, expected);

      RotationMatrix actual = new RotationMatrix();
      state.getRotation(actual);

      EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, 1.0e-9);


   }

   /** Contact columns are static in the world frame: predict must leave them unchanged. */
   @Test
   public void testContactsRemainStatic()
   {
      double dt = 1.0e-2;
      InvariantPropagator propagator = new InvariantPropagator(2, 1.0e-4, 1.0e-2, 1.0e-6, GRAVITY);
      InvariantState state = new InvariantState(2);

      Vector3D contact0 = new Vector3D(1.0, 2.0, 3.0);
      Vector3D contact1 = new Vector3D(-1.0, 0.0, 2.0);
      state.setContactPosition(0, contact0);
      state.setContactPosition(1, contact1);

      Vector3D omega = new Vector3D(0.1, 0.2, -0.1);
      Vector3D accel = new Vector3D(0.0, 0.0, -GRAVITY);
      propagator.predict(state, omega, accel, dt);

      Vector3D contactOut = new Vector3D();
      state.getContactPosition(0, contactOut);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(contact0, contactOut, EPSILON);
      state.getContactPosition(1, contactOut);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(contact1, contactOut, EPSILON);
   }

   /** With random inputs and noise, the covariance stays symmetric across many steps. */
   @Test
   public void testCovarianceStaysSymmetric()
   {
      Random random = new Random(12345L);
      double dt = 1.0e-2;
      InvariantPropagator propagator = new InvariantPropagator(2, 1.0e-3, 1.0e-2, 1.0e-4, GRAVITY);
      InvariantState state = new InvariantState(2);

      state.setRotation(EuclidCoreRandomTools.nextRotationMatrix(random));
      state.setBaseVelocity(EuclidCoreRandomTools.nextVector3D(random));
      state.setBasePosition(EuclidCoreRandomTools.nextVector3D(random));
      state.setContactPosition(0, EuclidCoreRandomTools.nextVector3D(random));
      state.setContactPosition(1, EuclidCoreRandomTools.nextVector3D(random));
      CommonOps_DDRM.setIdentity(state.getCovariance()); // start from a symmetric PSD P

      for (int i = 0; i < 50; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D accel = EuclidCoreRandomTools.nextVector3D(random);
         propagator.predict(state, omega, accel, dt);

         assertSymmetric(state.getCovariance(), 1.0e-9);
      }
   }

   /** Starting from P = 0, one predict yields P = Q_d: symmetric with positive trace. */
   @Test
   public void testCovarianceGrowsFromZero()
   {
      double dt = 1.0e-2;
      InvariantPropagator propagator = new InvariantPropagator(1, 1.0e-3, 1.0e-2, 1.0e-4, GRAVITY);
      InvariantState state = new InvariantState(1); // P starts at zero

      Vector3D omega = new Vector3D(0.0, 0.0, 0.0);
      Vector3D accel = new Vector3D(0.0, 0.0, -GRAVITY);
      propagator.predict(state, omega, accel, dt);

      DMatrixRMaj covariance = state.getCovariance();
      assertSymmetric(covariance, 1.0e-12);
      assertTrue(CommonOps_DDRM.trace(covariance) > 0.0);
   }

   /** With zero process noise and P = 0, the covariance stays exactly zero. */
   @Test
   public void testZeroNoiseKeepsCovarianceZero()
   {
      double dt = 1.0e-2;
      InvariantPropagator propagator = new InvariantPropagator(1, 0.0, 0.0, 0.0, GRAVITY);
      InvariantState state = new InvariantState(1); // P starts at zero

      Vector3D omega = new Vector3D(0.0, 0.0, 0.0);
      Vector3D accel = new Vector3D(0.0, 0.0, -GRAVITY);
      for (int i = 0; i < 20; i++)
         propagator.predict(state, omega, accel, dt);

      assertTrue(MatrixFeatures_DDRM.isZeros(state.getCovariance(), EPSILON));
   }

   /**
    * Log-linear property (the defining property of the invariant EKF): for two trajectories driven by
    * identical inputs with no noise, the right-invariant error η = X̂·X⁻¹ in exponential coordinates
    * evolves EXACTLY linearly, ξ_N = exp(A·T)·ξ₀, even for a large initial error. exp(A·T) has the same
    * block structure as the per-step Φ, using the total elapsed time T.
    */
   @Test
   public void testLogLinearErrorPropagation()
   {
      Random random = new Random(2024L);
      int numberOfContacts = 2;
      int n = 5 + numberOfContacts; // group size
      int m = 9 + 3 * numberOfContacts; // tangent space size
      double dt = 1.0e-3;
      int steps = 200;

      // true initial state - a random, valid SE_k(3) element
      InvariantState trueState = new InvariantState(numberOfContacts);
      trueState.setRotation(EuclidCoreRandomTools.nextRotationMatrix(random));
      trueState.setBaseVelocity(EuclidCoreRandomTools.nextVector3D(random));
      trueState.setBasePosition(EuclidCoreRandomTools.nextVector3D(random));
      for (int i = 0; i < numberOfContacts; i++)
         trueState.setContactPosition(i, EuclidCoreRandomTools.nextVector3D(random));

      // --- a deliberately LARGE initial error ξ₀ (log-linearity is exact, not just first-order) ---

      double[] xi0 = new double[m];
      for (int j = 0; j < m; j++)
         xi0[j] = 0.5 * (2.0 * random.nextDouble() - 1.0); // ~ U(-0.5, 0.5)

      // X̂₀ = exp_G(ξ₀) · X₀, so the right-invariant error η₀ = X̂₀·X₀⁻¹ = exp_G(ξ₀).
      DMatrixRMaj expXi0 = new DMatrixRMaj(n,n);
      SEK3Utils.exp(xi0, expXi0);
      DMatrixRMaj estimateInitial = new DMatrixRMaj(n,n);
      CommonOps_DDRM.mult(expXi0, trueState.getGroupElement(), estimateInitial);

      InvariantState estimateState = new InvariantState(numberOfContacts);
      estimateState.getGroupElement().set(estimateInitial); // copy into the live group matrix

      // propagate both with identical inputs and no process noise
      InvariantPropagator propagator = new InvariantPropagator(numberOfContacts, 0.0, 0.0, 0.0, GRAVITY);
      Vector3D omega = new Vector3D(0.4, -0.3, 0.6);
      Vector3D accel = new Vector3D(0.5, 0.2, -GRAVITY);
      for (int i = 0; i < steps; i++)
      {
         propagator.predict(trueState, omega, accel, dt);
         propagator.predict(estimateState, omega, accel, dt);
      }

      // --- measured final error: ξ_N = log_G(X̂_N · X_N⁻¹) ---
      DMatrixRMaj trueInverse = new DMatrixRMaj(n,n);
      CommonOps_DDRM.invert(trueState.getGroupElement(), trueInverse);
      DMatrixRMaj etaFinal = new DMatrixRMaj(n,n);
      CommonOps_DDRM.mult(estimateState.getGroupElement(), trueInverse, etaFinal);
      double[] xiMeasured = new double[m];
      SEK3Utils.log(etaFinal, xiMeasured);

      // --- predicted final error: ξ_N = exp(A·T) · ξ₀ ---
      double time = steps * dt;
      DMatrixRMaj errorTransition = new DMatrixRMaj(m,m);
      buildErrorTransition(time, errorTransition);
      DMatrixRMaj xi0Column = new DMatrixRMaj(m, 1, true, xi0);
      DMatrixRMaj xiPredicted = new DMatrixRMaj(m,1);
      CommonOps_DDRM.mult(errorTransition, xi0Column, xiPredicted);

      for (int j = 0; j < m; j++)
         assertEquals(xiPredicted.get(j,0),xiMeasured[j], 1.0e-10);

   }

   /** Builds the error transition exp(A·time) (same blocks as Φ, using the elapsed time). */
   private static void buildErrorTransition(double time, DMatrixRMaj transitionToPack)
   {
      CommonOps_DDRM.setIdentity(transitionToPack);

      Vector3D gravity = new Vector3D(0.0, 0.0, GRAVITY);
      Matrix3D hatGravity = new Matrix3D();
      SO3LieGroupTools.hat(gravity, hatGravity);

      // same block structure as the per-step Φ, but using the total elapsed time
      for (int r = 0; r < 3; r++)
      {
         for (int c = 0; c < 3; c++)
         {
            transitionToPack.set(3 + r, c, time * hatGravity.getElement(r, c));
            transitionToPack.set(6 + r, c, 0.5 * time * time * hatGravity.getElement(r, c));
         }
      }
      for (int j = 0; j < 3; j++)
         transitionToPack.add(6 + j, 3 + j, time); // (p, v) = I * time

   }

   /** Asserts P_ij == P_ji within eps. */
   private static void assertSymmetric(DMatrixRMaj matrix, double epsilon)
   {
      for (int r = 0; r < matrix.getNumRows(); r++)
         for (int c = r + 1; c < matrix.getNumCols(); c++)
            assertEquals(matrix.get(r,c), matrix.get(c,r),epsilon);
   }
}
