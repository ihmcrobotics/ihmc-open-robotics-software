package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertAllClose;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.block;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

/**
 * Locks in the mass-matrix-induced process noise of the write-up, eqs. (10)–(12): the unmodeled torque
 * {@code w_τ} maps to joint acceleration through {@code M(q)⁻¹}, so with scalar {@code Σ_τ = σ_τ² I} and
 * symmetric {@code M}, {@code Qa = σ_τ² M(q)⁻²}, discretized through the same Van Loan closed form
 * ({@code dt³/3 · Qa}, {@code dt²/2 · Qa}, {@code dt · Qa}) as the scalar path.
 *
 * <p>The expected values are computed INDEPENDENTLY of the filter: a second
 * {@link CompositeRigidBodyMassMatrixCalculator} over the same live chain and filtered joints, inverted with
 * plain EJML, never touching the filter's own solver or column mapping. The scalar-fallback behavior (no
 * robot model) is locked in by {@link JointLevelKFTransitionNoiseTest}; here we only assert the two paths
 * actually differ.</p>
 */
public class JointLevelKFMassMatrixNoiseTest
{
   private static final double DT = JointLevelKFTestFixture.DT;
   private static final double SIGMA_TAU = 50.0;   // matches JointLevelKFPreFilter.SIGMA_TAU
   private static final double SIGMA_ACCEL = 50.0; // matches JointLevelKFPreFilter.SIGMA_ACCEL (fallback path)

   /**
    * Relative tolerance for comparisons against the reference Qa: the filter inverts M by Cholesky, the
    * reference by LU, so they agree only to round-off amplified by M's conditioning — and M⁻² entries can be
    * numerically large for light random links, so a fixed absolute tolerance is meaningless.
    */
   private static double relTol(DMatrixRMaj expected)
   {
      return 1.0e-8 * Math.max(1.0e-30, CommonOps_DDRM.elementMaxAbs(expected));
   }

   /** Independent reference: Qa = σ_τ² (M⁻¹)², with M over the filtered joints in filter state order. */
   private static DMatrixRMaj referenceQa(JointLevelKFTestFixture f)
   {
      int n = f.n;
      List<JointReadOnly> joints = new ArrayList<>(f.filteredJoints);
      // Single-list overload = joints to CONSIDER (the (rootBody, joints) overload takes joints to IGNORE).
      MultiBodySystemReadOnly input = MultiBodySystemReadOnly.toMultiBodySystemInput(joints);
      CompositeRigidBodyMassMatrixCalculator calc = new CompositeRigidBodyMassMatrixCalculator(input);
      calc.reset();
      DMatrixRMaj massMatrix = calc.getMassMatrix().copy();
      assertEquals(n, massMatrix.numRows, "reference mass matrix is n x n");

      DMatrixRMaj massMatrixInverse = new DMatrixRMaj(n, n);
      assertTrue(CommonOps_DDRM.invert(massMatrix, massMatrixInverse), "reference mass matrix inverts");
      DMatrixRMaj minvSq = new DMatrixRMaj(n, n);
      CommonOps_DDRM.mult(massMatrixInverse, massMatrixInverse, minvSq);

      // Permute from the calculator's DoF ordering into filter state order, symmetrizing exactly as the
      // filter does so equality holds to round-off.
      int[] col = new int[n];
      for (int i = 0; i < n; i++)
      {
         OneDoFJointBasics joint = f.filteredJoints.get(i);
         col[i] = input.getJointMatrixIndexProvider().getJointDoFIndices(joint)[0];
      }
      DMatrixRMaj qa = new DMatrixRMaj(n, n);
      double st2 = SIGMA_TAU * SIGMA_TAU;
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            qa.set(i, j, st2 * 0.5 * (minvSq.get(col[i], col[j]) + minvSq.get(col[j], col[i])));
      return qa;
   }

   @Test
   public void testMassMatrixPathEnabledOnlyWithModel()
   {
      assertTrue(JointLevelKFTestFixture.singlePairMassMatrix(6000L, 8, 1, 7).filter.isUsingMassMatrixProcessNoise(),
                 "model provided => mass-matrix path");
      assertFalse(JointLevelKFTestFixture.singlePair(6000L, 8, 1, 7).filter.isUsingMassMatrixProcessNoise(),
                  "no model => scalar fallback");
   }

   @Test
   public void testProcessNoiseEqualsVanLoanOfMassMatrixInverseSquared()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(6100L))
      {
         int n = f.n;
         DMatrixRMaj qa = referenceQa(f);
         DMatrixRMaj Q = f.filter.getProcessNoise();

         DMatrixRMaj expected = new DMatrixRMaj(n, n);
         CommonOps_DDRM.scale(DT * DT * DT / 3.0, qa, expected);
         assertAllClose(block(Q, 0, 0, n, n), expected, relTol(expected), f.describe() + " Q qq = dt³/3 σ_τ² M⁻²");
         CommonOps_DDRM.scale(DT * DT / 2.0, qa, expected);
         assertAllClose(block(Q, 0, n, n, n), expected, relTol(expected), f.describe() + " Q q,q̇ = dt²/2 σ_τ² M⁻²");
         assertAllClose(block(Q, n, 0, n, n), expected, relTol(expected), f.describe() + " Q q̇,q = dt²/2 σ_τ² M⁻²");
         CommonOps_DDRM.scale(DT, qa, expected);
         assertAllClose(block(Q, n, n, n, n), expected, relTol(expected), f.describe() + " Q q̇q̇ = dt σ_τ² M⁻²");
      }
   }

   @Test
   public void testBiasBlockUntouchedByMassMatrixPath()
   {
      // The mass-matrix update rewrites ONLY the joint blocks: the per-IMU bias random walk and the zero
      // joint/bias coupling must be identical to the scalar path's construction.
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(6200L))
      {
         int n = f.n;
         int mDof = 3 * f.m;
         DMatrixRMaj Q = f.filter.getProcessNoise();
         assertAllClose(block(Q, 2 * n, 2 * n, mDof, mDof),
                        JointLevelKFTestFixture.scaledIdentity(mDof, DT * JointLevelKFTestFixture.IMU_BIAS_PROCESS_VAR),
                        1.0e-12,
                        f.describe() + " Q bias = dt·Σ_imu");
         assertAllClose(block(Q, 0, 2 * n, 2 * n, mDof), new DMatrixRMaj(2 * n, mDof), 1.0e-12, f.describe() + " Q joint,bias = 0");
         assertAllClose(block(Q, 2 * n, 0, mDof, 2 * n), new DMatrixRMaj(mDof, 2 * n), 1.0e-12, f.describe() + " Q bias,joint = 0");
      }
   }

   @Test
   public void testProcessNoiseSymmetricPSDOnMassMatrixPath()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(6300L))
      {
         DMatrixRMaj Q = f.filter.getProcessNoise();
         assertSymmetric(Q, 1.0e-12, f.describe() + " Q symmetric");
         assertPositiveSemiDefinite(Q, f.describe() + " Q PSD");
      }
   }

   @Test
   public void testProcessNoiseCouplesJointsThroughInertia()
   {
      // The whole point of eq. (11): M⁻² is dense for a serial chain, so unmodeled torque at one joint
      // correlates the process noise across joints. The scalar path is strictly (block-)diagonal.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairMassMatrix(6400L, 10, 1, 9); // n = 8
      int n = f.n;
      DMatrixRMaj qdBlock = block(f.filter.getProcessNoise(), n, n, n, n);
      double maxOffDiagonal = 0.0;
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            if (i != j)
               maxOffDiagonal = Math.max(maxOffDiagonal, Math.abs(qdBlock.get(i, j)));
      assertTrue(maxOffDiagonal > 0.0, "mass-matrix Qa couples joints (dense M⁻² ⇒ nonzero off-diagonals)");

      JointLevelKFTestFixture scalar = JointLevelKFTestFixture.singlePair(6400L, 10, 1, 9);
      DMatrixRMaj scalarQdBlock = block(scalar.filter.getProcessNoise(), n, n, n, n);
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            if (i != j)
               assertEquals(0.0, scalarQdBlock.get(i, j), 0.0, "scalar path stays diagonal");
   }

   @Test
   public void testProcessNoiseIsConfigurationDependent()
   {
      // M depends on q, so predict() must refresh Q when the model moves. Drive the live chain to a clearly
      // different configuration, refresh, and require Q to change — then require it to again match the
      // independent reference at the NEW configuration (i.e. it tracked, not just drifted).
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairMassMatrix(6500L, 10, 1, 9);
      int n = f.n;
      DMatrixRMaj qBefore = block(f.filter.getProcessNoise(), n, n, n, n);

      double[] q = new double[n];
      double[] qd = new double[n];
      for (int i = 0; i < n; i++)
         q[i] = 0.9 * ((i % 2 == 0) ? 1.0 : -1.0); // large alternating bend: far from the random build pose
      f.applyConsistentMotion(q, qd); // sets the live joints + updates frames

      f.filter.updateProcessNoiseFromMassMatrixForTest();
      DMatrixRMaj qAfter = block(f.filter.getProcessNoise(), n, n, n, n);

      double maxChange = 0.0;
      for (int i = 0; i < n * n; i++)
         maxChange = Math.max(maxChange, Math.abs(qAfter.get(i) - qBefore.get(i)));
      double scale = Math.max(1.0e-30, CommonOps_DDRM.elementMaxAbs(qBefore));
      assertTrue(maxChange > 1.0e-6 * scale, "Q must change with configuration (relative change " + maxChange / scale + ")");

      // And it must equal the reference recomputed at the new configuration.
      DMatrixRMaj qa = referenceQa(f);
      DMatrixRMaj expected = new DMatrixRMaj(n, n);
      CommonOps_DDRM.scale(DT, qa, expected);
      assertAllClose(qAfter, expected, relTol(expected), "Q q̇q̇ tracks M(q) at the new configuration");
   }

   @Test
   public void testPredictRefreshesQAndKeepsCovarianceSymmetricPSD()
   {
      // predict() itself must do the refresh (not just the test hook), and the refreshed Q must keep P
      // symmetric PSD through the propagation.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairMassMatrix(6600L, 8, 1, 7);
      int n = f.n;
      DMatrixRMaj x0 = new DMatrixRMaj(f.dim, 1);
      DMatrixRMaj p0 = JointLevelKFTestFixture.spd(f.dim, 42L);
      f.filter.setStateForTest(x0, p0);

      double[] q = new double[n];
      double[] qd = new double[n];
      for (int i = 0; i < n; i++)
         q[i] = -0.8 + 0.2 * i;
      f.applyConsistentMotion(q, qd);

      f.filter.predict();

      DMatrixRMaj qa = referenceQa(f); // reference at the post-motion configuration predict() should have used
      DMatrixRMaj expected = new DMatrixRMaj(n, n);
      CommonOps_DDRM.scale(DT, qa, expected);
      assertAllClose(block(f.filter.getProcessNoise(), n, n, n, n), expected, relTol(expected), "predict() refreshed Q from M(q)");

      DMatrixRMaj P = f.filter.getCovariance();
      // Relative symmetry tolerance: P's magnitude is set by Q's (σ_τ² M⁻² can be numerically large), and
      // F P Fᵀ is only round-off symmetric.
      assertSymmetric(P, 1.0e-9 * Math.max(1.0, CommonOps_DDRM.elementMaxAbs(P)), "P symmetric after mass-matrix predict");
      assertPositiveSemiDefinite(P, "P PSD after mass-matrix predict");
   }

   @Test
   public void testMassMatrixAndScalarPathsDiffer()
   {
      // Same seed, same chain shape: the two paths must produce genuinely different joint noise (otherwise
      // the wiring is silently on the fallback).
      JointLevelKFTestFixture mass = JointLevelKFTestFixture.singlePairMassMatrix(6700L, 8, 1, 7);
      JointLevelKFTestFixture scalar = JointLevelKFTestFixture.singlePair(6700L, 8, 1, 7);
      int n = mass.n;
      DMatrixRMaj qMass = block(mass.filter.getProcessNoise(), n, n, n, n);
      DMatrixRMaj qScalar = block(scalar.filter.getProcessNoise(), n, n, n, n);
      double sa2dt = SIGMA_ACCEL * SIGMA_ACCEL * DT;
      assertEquals(sa2dt, qScalar.get(0, 0), 1.0e-12, "scalar path sanity");
      double maxDiff = 0.0;
      for (int i = 0; i < n * n; i++)
         maxDiff = Math.max(maxDiff, Math.abs(qMass.get(i) - qScalar.get(i)));
      assertTrue(maxDiff > 1.0e-3 * sa2dt, "mass-matrix Q differs from scalar Q");
   }
}
