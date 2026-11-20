package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * The kinematic-velocity measurement error block ({@link KinematicVelocityErrorModel}): a first-order Gauss-Markov
 * error {@code b_v} in {@code y = Rᵀv + b_v + n}, carried either as an estimated state or as a Schmidt consider state.
 */
public class InvariantEKFKinematicVelocityErrorTest
{
   private static final int CONTACTS = 2;
   private static final double TAU = 0.1;
   private static final double SIGMA = 0.03;

   private static InvariantEKF filter(boolean withBlock)
   {
      return new InvariantEKF(CONTACTS, 1.0e-4, 1.0e-3, 1.0e-4, -9.81, true, withBlock);
   }

   private static void seed(InvariantEKF ekf, RotationMatrix rotation, Vector3D velocity, double variance)
   {
      ekf.getState().setRotation(rotation);
      ekf.getState().setBaseVelocity(velocity);
      DMatrixRMaj P = ekf.getState().getCovariance();
      CommonOps_DDRM.setIdentity(P);
      CommonOps_DDRM.scale(variance, P);
      int bv = ekf.getState().kinematicVelocityBiasTangentIndex();
      if (bv >= 0)
         for (int i = 0; i < 3; i++)
            P.set(bv + i, bv + i, ekf.isKinematicVelocityErrorModelActive() ? SIGMA * SIGMA : 0.0);
   }

   private static Matrix3D isotropic(double variance)
   {
      Matrix3D m = new Matrix3D();
      m.setIdentity();
      m.scale(variance);
      return m;
   }

   @Test
   public void testTheBlockSitsAfterTheGyroBias()
   {
      InvariantState state = new InvariantState(CONTACTS, true, true);
      assertEquals(15 + 3 * CONTACTS, state.getTangentSize());
      assertEquals(9 + 3 * CONTACTS, state.gyroBiasTangentIndex());
      assertEquals(12 + 3 * CONTACTS, state.kinematicVelocityBiasTangentIndex());
      assertEquals(9 + 3 * CONTACTS, new InvariantState(CONTACTS, false, true).kinematicVelocityBiasTangentIndex());
      assertEquals(-1, new InvariantState(CONTACTS, true, false).kinematicVelocityBiasTangentIndex());
   }

   @Test
   public void testAnInactiveBlockLeavesTheFilterExactlyAsWithoutIt()
   {
      Random random = new Random(11L);
      InvariantEKF with = filter(true);
      InvariantEKF without = filter(false);
      RotationMatrix rotation = new RotationMatrix(EuclidCoreRandomTools.nextQuaternion(random));
      Vector3D velocity = EuclidCoreRandomTools.nextVector3D(random, 0.5);
      seed(with, rotation, velocity, 0.1);
      seed(without, rotation, velocity, 0.1);
      for (int tick = 0; tick < 50; tick++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextVector3D(random, 0.3);
         Vector3D accel = EuclidCoreRandomTools.nextVector3D(random, 1.0);
         accel.addZ(9.81);
         with.predict(omega, accel, 1.0e-3);
         without.predict(omega, accel, 1.0e-3);
         Vector3D y = EuclidCoreRandomTools.nextVector3D(random, 0.5);
         Vector3D lever = EuclidCoreRandomTools.nextVector3D(random, 1.0);
         with.updateBodyVelocity(y, isotropic(0.01), lever);
         without.updateBodyVelocity(y, isotropic(0.01), lever);
      }
      int m = without.getState().getTangentSize();
      DMatrixRMaj P = with.getState().getCovariance();
      for (int i = 0; i < m; i++)
         for (int j = 0; j < m; j++)
            assertEquals(without.getState().getCovariance().get(i, j), P.get(i, j), 0.0, "P(" + i + "," + j + ")");
      for (int i = 0; i < with.getState().getGroupElement().getNumElements(); i++)
         assertEquals(without.getState().getGroupElement().get(i), with.getState().getGroupElement().get(i), 0.0);
      for (int i = m; i < P.getNumRows(); i++)
         for (int j = 0; j < P.getNumCols(); j++)
            assertEquals(0.0, P.get(i, j), 0.0, "decoupled block P(" + i + "," + j + ")");
   }

   @Test
   public void testWithoutMeasurementsTheBlockSettlesToItsStationaryVarianceAndItsMeanDecays()
   {
      InvariantEKF ekf = filter(true);
      ekf.setKinematicVelocityErrorModel(true, false, TAU, SIGMA);
      seed(ekf, new RotationMatrix(), new Vector3D(), 0.01);
      ekf.getKinematicVelocityBias().set(0.02, -0.01, 0.0);
      int bv = ekf.getState().kinematicVelocityBiasTangentIndex();
      for (int tick = 0; tick < 1000; tick++)
         ekf.predict(new Vector3D(), new Vector3D(0.0, 0.0, 9.81), 1.0e-3);
      for (int i = 0; i < 3; i++)
         assertEquals(SIGMA * SIGMA, ekf.getState().getCovariance().get(bv + i, bv + i), 0.02 * SIGMA * SIGMA);
      // The mean decays with the process: one second is ten time constants.
      assertEquals(0.02 * Math.exp(-1.0 / TAU), ekf.getKinematicVelocityBias().getX(), 1.0e-9);
   }

   @Test
   public void testTheJacobianMatchesTheResidual()
   {
      // r(b̂) = R̂(y − b̂) − v̂ ; moving b̂ by +ε along body axis k must change r by −R̂·e_k·ε, and H = −R̂ on the block
      // with δb = b̂ − b means r ≈ H·δb ... so ∂r/∂b̂ = −R̂ is exactly H's block.
      Random random = new Random(5L);
      RotationMatrix rotation = new RotationMatrix(EuclidCoreRandomTools.nextQuaternion(random));
      InvariantEKF a = filter(true);
      a.setKinematicVelocityErrorModel(true, false, TAU, SIGMA);
      seed(a, rotation, new Vector3D(0.2, 0.1, 0.0), 1.0e-2);
      Vector3D y = new Vector3D(0.3, -0.2, 0.1);
      // Tiny measurement noise and a dominant bias prior: the update attributes the residual to the bias block in the
      // direction −R̂ predicts, so b̂ moves by ≈ −(R̂ᵀ r) when the velocity prior is much tighter.
      DMatrixRMaj P = a.getState().getCovariance();
      CommonOps_DDRM.scale(1.0e-8, P);
      int bv = a.getState().kinematicVelocityBiasTangentIndex();
      for (int i = 0; i < 3; i++)
         P.set(bv + i, bv + i, 1.0);
      Vector3D vWorld = new Vector3D();
      a.getState().getBaseVelocity(vWorld);
      Vector3D vBody = new Vector3D();
      rotation.inverseTransform(vWorld, vBody);
      a.updateBodyVelocity(y, isotropic(1.0e-10), null);
      Vector3D expected = new Vector3D(y);
      expected.sub(vBody); // the whole disagreement is the measurement's error
      assertTrue(a.getKinematicVelocityBias().epsilonEquals(expected, 1.0e-5), a.getKinematicVelocityBias() + " vs " + expected);
   }

   @Test
   public void testConsiderModeNeverMovesTheErrorButWidensTheVelocityCovariance()
   {
      Random random = new Random(7L);
      InvariantEKF white = filter(true);
      InvariantEKF consider = filter(true);
      consider.setKinematicVelocityErrorModel(true, true, TAU, SIGMA);
      RotationMatrix rotation = new RotationMatrix(EuclidCoreRandomTools.nextQuaternion(random));
      seed(white, rotation, new Vector3D(), 0.01);
      seed(consider, rotation, new Vector3D(), 0.01);
      for (int tick = 0; tick < 500; tick++)
      {
         white.predict(new Vector3D(), new Vector3D(0.0, 0.0, 9.81), 1.0e-3);
         consider.predict(new Vector3D(), new Vector3D(0.0, 0.0, 9.81), 1.0e-3);
         Vector3D y = EuclidCoreRandomTools.nextVector3D(random, 0.01);
         white.updateBodyVelocity(y, isotropic(1.0e-3), null);
         consider.updateBodyVelocity(y, isotropic(1.0e-3), null);
      }
      assertEquals(0.0, consider.getKinematicVelocityBias().norm(), 0.0);
      int v = white.getState().baseVelocityTangentIndex();
      double whiteVar = white.getState().getCovariance().get(v, v);
      double considerVar = consider.getState().getCovariance().get(v, v);
      // 500 white updates shrink the velocity variance far below the correlated 3 cm/s error; consider keeps it near σ.
      assertTrue(considerVar > 10.0 * whiteVar, "consider " + considerVar + " vs white " + whiteVar);
      assertTrue(Math.sqrt(considerVar) > 0.5 * SIGMA, "consider velocity std " + Math.sqrt(considerVar));
   }

   @Test
   public void testTheStateModeTracksASlowMeasurementOffset()
   {
      // The truth is still; the measurement carries a +2 cm/s offset that moves slowly compared with the IMU's
      // ability to say "not accelerating". The estimated error should absorb a large part of it and the velocity
      // estimate should stay closer to zero than with the white model.
      InvariantEKF white = filter(true);
      InvariantEKF state = filter(true);
      state.setKinematicVelocityErrorModel(true, false, 1.0, SIGMA);
      seed(white, new RotationMatrix(), new Vector3D(), 1.0e-4);
      seed(state, new RotationMatrix(), new Vector3D(), 1.0e-4);
      Vector3D y = new Vector3D(0.02, 0.0, 0.0);
      for (int tick = 0; tick < 300; tick++)
      {
         white.predict(new Vector3D(), new Vector3D(0.0, 0.0, 9.81), 1.0e-3);
         state.predict(new Vector3D(), new Vector3D(0.0, 0.0, 9.81), 1.0e-3);
         white.updateBodyVelocity(y, isotropic(1.0e-4), null);
         state.updateBodyVelocity(y, isotropic(1.0e-4), null);
      }
      Vector3D vWhite = new Vector3D(), vState = new Vector3D();
      white.getState().getBaseVelocity(vWhite);
      state.getState().getBaseVelocity(vState);
      assertTrue(Math.abs(vState.getX()) < 0.5 * Math.abs(vWhite.getX()), "state " + vState.getX() + " vs white " + vWhite.getX());
      assertTrue(state.getKinematicVelocityBias().getX() > 0.01, "estimated error " + state.getKinematicVelocityBias().getX());
   }
}
