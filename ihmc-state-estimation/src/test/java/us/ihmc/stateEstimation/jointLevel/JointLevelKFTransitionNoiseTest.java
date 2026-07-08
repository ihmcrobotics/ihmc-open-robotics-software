package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertAllClose;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.block;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.identity;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.scaledIdentity;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

/**
 * Ported from {@code tests/jointKF/test_noise.py}. Locks in the transition matrix F, the discrete process
 * noise Q (continuous white-noise-acceleration / Van Loan closed form), and the encoder measurement noise.
 *
 * <p>Adaptations to this implementation: these tests exercise the diagonal {@code σ_acc² I} FALLBACK path
 * ({@code SIGMA_ACCEL = 50}, no robot model handed to the filter). The mass-matrix ({@code σ_τ² M⁻²}) path is
 * covered separately in {@link JointLevelKFMassMatrixNoiseTest}. The bias random walk is per-IMU, sourced from
 * each IMU's own angular-velocity bias process-noise
 * covariance ({@code Q_bias = dt · Σ_imu}), not a single scalar {@code σ_b² dt I}. The gyro measurement noise
 * is assembled per pair (covered in {@link JointLevelKFMeasurementTest}); here we lock in the encoder block.</p>
 */
public class JointLevelKFTransitionNoiseTest
{
   private static final double DT = JointLevelKFTestFixture.DT;
   private static final double SIGMA_ACCEL = 50.0;   // matches JointLevelKFPreFilter.SIGMA_ACCEL
   private static final double ENCODER_VAR = 1.0e-6; // matches JointLevelKFPreFilter.ENCODER_VAR
   private static final double IMU_BIAS_VAR = JointLevelKFTestFixture.IMU_BIAS_PROCESS_VAR;

   @Test
   public void testBuildFStructureAndExactness()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(1000L))
      {
         int n = f.n;
         int dim = f.dim;
         DMatrixRMaj F = f.filter.getTransitionMatrix();
         assertEquals(dim, F.numRows, f.describe());
         // Diagonal blocks are identity.
         assertAllClose(block(F, 0, 0, n, n), identity(n), 1.0e-12, f.describe() + " F qq");
         assertAllClose(block(F, n, n, n, n), identity(n), 1.0e-12, f.describe() + " F q̇q̇");
         assertAllClose(block(F, 2 * n, 2 * n, 3 * f.m, 3 * f.m), identity(3 * f.m), 1.0e-12, f.describe() + " F bb");
         // q -> q̇ coupling is exactly dt·I.
         assertAllClose(block(F, 0, n, n, n), scaledIdentity(n, DT), 1.0e-12, f.describe() + " F q,q̇");
         // Everything off the block diagonal (below and the bias couplings) is zero.
         assertAllClose(block(F, n, 0, n, n), new DMatrixRMaj(n, n), 1.0e-12, f.describe() + " F q̇,q");
         assertAllClose(block(F, 2 * n, 0, 3 * f.m, 2 * n), new DMatrixRMaj(3 * f.m, 2 * n), 1.0e-12, f.describe() + " F b,joint");
         assertAllClose(block(F, 0, 2 * n, 2 * n, 3 * f.m), new DMatrixRMaj(2 * n, 3 * f.m), 1.0e-12, f.describe() + " F joint,b");
      }
   }

   @Test
   public void testBuildFEqualsIPlusADt()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(1001L, 6, 1, 4); // n = 3
      int n = f.n;
      int dim = f.dim;
      DMatrixRMaj F = f.filter.getTransitionMatrix();
      DMatrixRMaj expected = identity(dim); // I
      for (int i = 0; i < n; i++)
         expected.set(i, n + i, DT); // + A·dt, A[:n, n:2n] = I
      assertAllClose(F, expected, 1.0e-12, "F = I + A·dt");
   }

   @Test
   public void testProcessNoiseVanLoanJointBlocks()
   {
      double sa2 = SIGMA_ACCEL * SIGMA_ACCEL;
      DMatrixRMaj Qa = scaledIdentity(1, sa2); // per-joint acceleration variance (diagonal path)
      double posFactor = DT * DT * DT / 3.0;
      double crossFactor = DT * DT / 2.0;
      double velFactor = DT;

      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(1100L))
      {
         int n = f.n;
         DMatrixRMaj Q = f.filter.getProcessNoise();
         // Van Loan sub-blocks for a double integrator with white acceleration Q_a = σ_acc² I.
         assertAllClose(block(Q, 0, 0, n, n), scaledIdentity(n, posFactor * sa2), 1.0e-9, f.describe() + " Q qq = dt³/3 σ²");
         assertAllClose(block(Q, 0, n, n, n), scaledIdentity(n, crossFactor * sa2), 1.0e-9, f.describe() + " Q q,q̇ = dt²/2 σ²");
         assertAllClose(block(Q, n, 0, n, n), scaledIdentity(n, crossFactor * sa2), 1.0e-9, f.describe() + " Q q̇,q = dt²/2 σ²");
         assertAllClose(block(Q, n, n, n, n), scaledIdentity(n, velFactor * sa2), 1.0e-9, f.describe() + " Q q̇q̇ = dt σ²");
         assertTrue(Qa.get(0, 0) > 0.0);
      }
   }

   @Test
   public void testProcessNoiseBiasBlockAndNoCrossCoupling()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(1200L))
      {
         int n = f.n;
         int mDof = 3 * f.m;
         DMatrixRMaj Q = f.filter.getProcessNoise();
         // Bias random walk: block-diagonal per IMU, dt · Σ_imu with Σ_imu = IMU_BIAS_VAR · I.
         assertAllClose(block(Q, 2 * n, 2 * n, mDof, mDof), scaledIdentity(mDof, DT * IMU_BIAS_VAR), 1.0e-12,
                        f.describe() + " Q bias = dt·Σ_imu");
         // No cross coupling between the joint and bias blocks.
         assertAllClose(block(Q, 0, 2 * n, 2 * n, mDof), new DMatrixRMaj(2 * n, mDof), 1.0e-12, f.describe() + " Q joint,bias = 0");
         assertAllClose(block(Q, 2 * n, 0, mDof, 2 * n), new DMatrixRMaj(mDof, 2 * n), 1.0e-12, f.describe() + " Q bias,joint = 0");
      }
   }

   @Test
   public void testProcessNoiseSymmetricPSD()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(1300L))
      {
         DMatrixRMaj Q = f.filter.getProcessNoise();
         assertSymmetric(Q, 1.0e-12, f.describe() + " Q symmetric");
         assertPositiveSemiDefinite(Q, f.describe() + " Q PSD");
      }
   }

   @Test
   public void testEncoderMeasurementModel()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(1400L))
      {
         int n = f.n;
         int dim = f.dim;
         DMatrixRMaj Henc = f.filter.getEncoderJacobian();
         DMatrixRMaj Renc = f.filter.getEncoderNoise();
         // Encoder Jacobian is [I_n | 0]: it observes exactly the position segment.
         assertEquals(n, Henc.numRows, f.describe());
         assertEquals(dim, Henc.numCols, f.describe());
         assertAllClose(block(Henc, 0, 0, n, n), identity(n), 1.0e-12, f.describe() + " Henc position block = I");
         assertAllClose(block(Henc, 0, n, n, dim - n), new DMatrixRMaj(n, dim - n), 1.0e-12, f.describe() + " Henc velocity/bias block = 0");
         // Encoder noise is σ_enc² I.
         assertAllClose(Renc, scaledIdentity(n, ENCODER_VAR), 1.0e-12, f.describe() + " Renc = σ_enc² I");
      }
   }
}
