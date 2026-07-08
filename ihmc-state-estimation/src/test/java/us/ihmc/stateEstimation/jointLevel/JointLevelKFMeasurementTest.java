package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertAllClose;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.block;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.identity;

import java.util.HashSet;
import java.util.Set;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Ported from {@code tests/jointKF/test_measurement.py}. Locks in the measurement model: the encoder Jacobian
 * [I_n | 0], and the per-pair relative-gyro measurement z = ω_child − R·ω_parent with Jacobian rows
 * [0 | scattered J | +R_child at child-bias | −R_parent at parent-bias].
 *
 * <p>This filter builds one measurement per IMU pair (sequential updates) rather than one stacked z/H, so the
 * reference's stacked {@code build_H}/{@code build_z} shape is checked here as the per-pair block structure.</p>
 */
public class JointLevelKFMeasurementTest
{
   @Test
   public void testEncoderJacobianStructure()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(4000L))
      {
         int n = f.n;
         DMatrixRMaj Henc = f.filter.getEncoderJacobian();
         assertAllClose(block(Henc, 0, 0, n, n), identity(n), 1.0e-12, f.describe() + " encoder rows observe q");
         assertAllClose(block(Henc, 0, n, n, f.dim - n), new DMatrixRMaj(n, f.dim - n), 1.0e-12,
                        f.describe() + " encoder rows independent of q̇ and bias");
      }
   }

   @Test
   public void testEncoderPredictsPosition()
   {
      // Predicted encoder measurement H_enc x = q (the position segment).
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4050L, 8, 1, 7);
      int n = f.n;
      DMatrixRMaj Henc = f.filter.getEncoderJacobian();
      DMatrixRMaj x = new DMatrixRMaj(f.dim, 1);
      for (int i = 0; i < f.dim; i++)
         x.set(i, 0, 0.1 * (i + 1));
      DMatrixRMaj hx = new DMatrixRMaj(n, 1);
      CommonOps_DDRM.mult(Henc, x, hx);
      for (int i = 0; i < n; i++)
         assertEquals(x.get(i, 0), hx.get(i, 0), 1.0e-12, "encoder predicts q[" + i + "]");
   }

   @Test
   public void testPairJacobianStructure()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(4100L))
      {
         int n = f.n;
         f.filter.buildPairMeasurementForTest(0);
         DMatrixRMaj H = f.filter.getMeasurementJacobian();
         assertEquals(3, H.numRows, f.describe());
         assertEquals(f.dim, H.numCols, f.describe());

         // Position (q) columns carry no measurement dependence.
         assertAllClose(block(H, 0, 0, 3, n), new DMatrixRMaj(3, n), 1.0e-12, f.describe() + " pair H q-block = 0");

         int parentBias = f.filter.getPairParentBiasColumn(0);
         int childBias = f.filter.getPairChildBiasColumn(0);
         Set<Integer> velocityColumns = new HashSet<>();
         for (int c : f.filter.getPairVelocityColumns(0))
            velocityColumns.add(c);

         // Every velocity column outside this pair's qdCols must be zero; bias columns outside this pair's two
         // IMU blocks must be zero.
         for (int col = n; col < 2 * n; col++)
            if (!velocityColumns.contains(col))
               for (int r = 0; r < 3; r++)
                  assertTrue(H.get(r, col) == 0.0, f.describe() + " unrelated velocity column " + col + " must be 0");
         for (int col = 2 * n; col < f.dim; col++)
         {
            boolean inParent = col >= parentBias && col < parentBias + 3;
            boolean inChild = col >= childBias && col < childBias + 3;
            if (!inParent && !inChild)
               for (int r = 0; r < 3; r++)
                  assertTrue(H.get(r, col) == 0.0, f.describe() + " unrelated bias column " + col + " must be 0");
         }
      }
   }

   @Test
   public void testChildBiasBlockIsIdentity()
   {
      // The child IMU's measurement frame is the Jacobian frame here, so R_child = I ⇒ +I in the child-bias block.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4200L, 8, 1, 7);
      f.filter.buildPairMeasurementForTest(0);
      DMatrixRMaj H = f.filter.getMeasurementJacobian();
      DMatrixRMaj childBlock = block(H, 0, f.filter.getPairChildBiasColumn(0), 3, 3);
      assertAllClose(childBlock, identity(3), 1.0e-9, "child-bias block = +R_child = I");
   }

   @Test
   public void testParentBiasBlockIsNegativeRotation()
   {
      // Parent-bias block is −R_parent, a negated rotation ⇒ its negative is orthonormal (RᵀR = I).
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4300L, 8, 1, 7);
      f.filter.buildPairMeasurementForTest(0);
      DMatrixRMaj H = f.filter.getMeasurementJacobian();
      DMatrixRMaj parentBlock = block(H, 0, f.filter.getPairParentBiasColumn(0), 3, 3);
      DMatrixRMaj rotation = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.scale(-1.0, parentBlock, rotation); // R_parent = −block
      DMatrixRMaj rtr = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.multTransA(rotation, rotation, rtr);
      assertAllClose(rtr, identity(3), 1.0e-9, "R_parentᵀ R_parent = I (it is a rotation)");
   }

   @Test
   public void testRelativeGyroParentZero()
   {
      // ω_parent = 0 ⇒ z = ω_child, expressed in the Jacobian frame (= the child's measurement frame here).
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4400L, 8, 1, 7);
      f.imus.get(0).setAngularVelocity(0.0, 0.0, 0.0);
      f.imus.get(1).setAngularVelocity(0.1, -0.2, 0.3);
      f.filter.buildPairMeasurementForTest(0);
      DMatrixRMaj z = f.filter.getMeasurementResidual();
      assertEquals(0.1, z.get(0, 0), 1.0e-9, "z_x = ω_child_x");
      assertEquals(-0.2, z.get(1, 0), 1.0e-9, "z_y = ω_child_y");
      assertEquals(0.3, z.get(2, 0), 1.0e-9, "z_z = ω_child_z");
   }

   @Test
   public void testRelativeGyroChildZeroPreservesNorm()
   {
      // ω_child = 0 ⇒ z = −R·ω_parent; a rotation preserves the norm.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4500L, 8, 1, 7);
      f.imus.get(0).setAngularVelocity(0.1, -0.2, 0.3);
      f.imus.get(1).setAngularVelocity(0.0, 0.0, 0.0);
      f.filter.buildPairMeasurementForTest(0);
      DMatrixRMaj z = f.filter.getMeasurementResidual();
      double zNorm = Math.sqrt(z.get(0, 0) * z.get(0, 0) + z.get(1, 0) * z.get(1, 0) + z.get(2, 0) * z.get(2, 0));
      double wNorm = Math.sqrt(0.1 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3);
      assertEquals(wNorm, zNorm, 1.0e-9, "|z| = |ω_parent| (rotation preserves norm)");
   }

   @Test
   public void testMeasurementNoiseSymmetricPSD()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(4600L))
      {
         f.filter.buildPairMeasurementForTest(0);
         DMatrixRMaj R = f.filter.getMeasurementNoise();
         assertEquals(3, R.numRows, f.describe());
         assertSymmetric(R, 1.0e-12, f.describe() + " pair R symmetric");
         assertPositiveSemiDefinite(R, f.describe() + " pair R PSD");
      }
   }

   @Test
   public void testMeasurementNoiseUsesGyroMeasurementCovariance()
   {
      // Regression for the bias-vs-measurement covariance mix-up: R must be assembled from each IMU's
      // angular-velocity MEASUREMENT noise (the write-up's R_ω, the per-IMU Mahony output covariance) rotated
      // into the Jacobian frame — R = R_c Σ_c R_cᵀ + R_p Σ_p R_pᵀ — and NOT from the (typically far smaller)
      // bias random-walk covariance, which made the pair-gyro update drastically over-confident.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4700L, 8, 1, 7);
      JointLevelKFTestFixture.TestIMU parent = f.imus.get(0);
      JointLevelKFTestFixture.TestIMU child = f.imus.get(1);

      // ANISOTROPIC measurement covariances (an isotropic σ²I is rotation-invariant and could not catch a
      // missing/wrong rotation), clearly distinct from the tiny bias process covariances.
      parent.setAngularVelocityNoiseCovarianceDiagonal(4.0e-4, 1.0e-6, 2.5e-5);
      child.setAngularVelocityNoiseCovarianceDiagonal(9.0e-4, 1.6e-5, 4.9e-6);
      parent.setBiasProcessNoiseCovarianceDiagonal(1.0e-9, 1.0e-9, 1.0e-9);
      child.setBiasProcessNoiseCovarianceDiagonal(1.0e-9, 1.0e-9, 1.0e-9);

      f.filter.buildPairMeasurementForTest(0);
      DMatrixRMaj R = f.filter.getMeasurementNoise();

      // Expected, built independently: the Jacobian frame is the child's body-fixed frame, so R_child = I and
      // R_parent is the parent-measurement-frame-to-child-frame rotation read straight off euclid.
      RigidBodyTransform parentToChild = new RigidBodyTransform();
      parent.getMeasurementFrame().getTransformToDesiredFrame(parentToChild, child.getMeasurementFrame());
      DMatrixRMaj rotParent = new DMatrixRMaj(3, 3);
      JointLevelKFPreFilter.set_matrix(rotParent, parentToChild.getRotation());

      DMatrixRMaj expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj sigmaParent = new DMatrixRMaj(3, 3);
      parent.getAngularVelocityNoiseCovariance(sigmaParent);
      DMatrixRMaj tmp = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(rotParent, sigmaParent, tmp);
      CommonOps_DDRM.multTransB(tmp, rotParent, expected); // R_p Σ_p R_pᵀ
      DMatrixRMaj sigmaChild = new DMatrixRMaj(3, 3);
      child.getAngularVelocityNoiseCovariance(sigmaChild);
      CommonOps_DDRM.addEquals(expected, sigmaChild);      // + I Σ_c Iᵀ

      assertAllClose(R, expected, 1.0e-12, "pair R = R_c Σ_c R_cᵀ + R_p Σ_p R_pᵀ from the MEASUREMENT covariances");

      // Explicit guard against the regression: had R been built from the bias process covariances, its trace
      // would be ~6e-9; the measurement-built R is orders of magnitude larger.
      double trace = R.get(0, 0) + R.get(1, 1) + R.get(2, 2);
      assertTrue(trace > 1.0e-5, "R clearly not built from the bias random-walk covariance (trace = " + trace + ")");
   }

   @Test
   public void testMeasurementNoiseIndependentOfBiasProcessCovariance()
   {
      // Scaling the bias random walk by 1000x must leave the pair measurement noise R untouched.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4800L, 8, 1, 7);
      f.filter.buildPairMeasurementForTest(0);
      DMatrixRMaj rBefore = f.filter.getMeasurementNoise();

      for (JointLevelKFTestFixture.TestIMU imu : f.imus)
         imu.setBiasProcessNoiseCovarianceDiagonal(1.0e-1, 1.0e-1, 1.0e-1);
      f.filter.buildPairMeasurementForTest(0);
      DMatrixRMaj rAfter = f.filter.getMeasurementNoise();

      assertAllClose(rAfter, rBefore, 0.0, "pair R independent of the bias process covariance");
   }
}
