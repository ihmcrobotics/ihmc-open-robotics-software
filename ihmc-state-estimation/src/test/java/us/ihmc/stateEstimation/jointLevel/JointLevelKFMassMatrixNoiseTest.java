package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertAllClose;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.block;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;

/**
 * Locks in the Schur-complement process noise of the SPEC (§3.2): an unmodeled joint torque {@code w_τ}
 * propagates through the FLOATING-BASE dynamics, and eliminating the unforced base acceleration gives
 * {@code δq̈ = Λ⁻¹ w_τ} with {@code Λ = M_jj − M_jb M_bb⁻¹ M_bj} the Schur complement of the 6×6 base block.
 * With scalar {@code Σ_τ = σ_τ² I} this is {@code Qa = σ_τ² Λ(q)⁻²}, discretized through the same Van Loan
 * closed form ({@code dt³/3 · Qa}, {@code dt²/2 · Qa}, {@code dt · Qa}) as the scalar path.
 *
 * <p>This replaces the Rev. 1 locked-base {@code Qa = σ_τ² M_jj⁻²} covered by the pre-Schur version of this
 * test: the reference below is updated to compute {@code Λ} rather than the joints-only {@code M_jj}, so the
 * (previously green) Van-Loan equivalence assertions now validate the Schur path exactly.</p>
 *
 * <p>The expected values are computed INDEPENDENTLY of the filter: a second
 * {@link CompositeRigidBodyMassMatrixCalculator} over the same live chain — built over the floating base plus
 * the filtered joints — with the block extraction, {@code M_bb} inverse, and Schur subtraction all done in
 * plain EJML (LU, not the filter's Cholesky), never touching the filter's own solvers or column mapping.
 * Disagreement to round-off localizes a block-extraction or sign error in the filter's Schur algebra. The
 * scalar-fallback behavior (no robot model) is locked in by {@link JointLevelKFTransitionNoiseTest}; here we
 * only assert the two paths actually differ.</p>
 */
public class JointLevelKFMassMatrixNoiseTest
{
   private static final double DT = JointLevelKFTestFixture.DT;
   private static final double SIGMA_TAU = 50.0;   // matches JointLevelKFPreFilter.SIGMA_TAU
   private static final double SIGMA_ACCEL = 50.0; // matches JointLevelKFPreFilter.SIGMA_ACCEL (fallback path)

   /**
    * Relative tolerance for comparisons against the reference Qa: the filter inverts by Cholesky, the
    * reference by LU, so they agree only to round-off amplified by the conditioning of {@code M_bb} and
    * {@code Λ} — and {@code Λ⁻²} entries can be numerically large for light random links, so a fixed absolute
    * tolerance is meaningless.
    */
   private static double relTol(DMatrixRMaj expected)
   {
      return 1.0e-8 * Math.max(1.0e-30, CommonOps_DDRM.elementMaxAbs(expected));
   }

   /**
    * Independent reference for the Schur complement {@code Λ} and the fully-locked filtered block {@code M_ff},
    * both in filter state order. Replicates the filter's model definition (SPEC §3.2 generalized to arbitrary
    * topology): builds a second calculator over the floating base + the joints spanning base→filtered, then
    * marginalizes the NUISANCE block (base 6-DoF + any unfiltered "gap" joints) via
    * {@code Λ = M_ff − M_Nf^T M_NN⁻¹ M_Nf} (using {@code M_jN = M_Nf^T} since {@code M} is symmetric). Done in
    * plain EJML (LU) — no contact with the filter's Cholesky solvers or column mapping. Returns {@code {Λ, M_ff}}.
    */
   private static DMatrixRMaj[] referenceSchur(JointLevelKFTestFixture f)
   {
      int n = f.n;

      // Spanning joints: walk up from each filtered joint to the floating base, collecting everything in between.
      LinkedHashSet<JointReadOnly> spanning = new LinkedHashSet<>();
      for (JointReadOnly filteredJoint : f.filteredJoints)
      {
         JointReadOnly joint = filteredJoint;
         while (joint != null && joint != f.rootJoint)
         {
            spanning.add(joint);
            joint = joint.getPredecessor().getParentJoint();
         }
      }
      List<JointReadOnly> consider = new ArrayList<>();
      consider.add(f.rootJoint); // the 6-DoF floating base — the whole point of the Schur switch
      consider.addAll(spanning);
      MultiBodySystemReadOnly input = MultiBodySystemReadOnly.toMultiBodySystemInput(consider);
      CompositeRigidBodyMassMatrixCalculator calc = new CompositeRigidBodyMassMatrixCalculator(input);
      calc.reset();
      DMatrixRMaj massMatrix = calc.getMassMatrix().copy();

      // Filtered-joint columns (in filter state order) and nuisance columns (base + gap joints).
      int[] filteredCols = new int[n];
      for (int i = 0; i < n; i++)
         filteredCols[i] = input.getJointMatrixIndexProvider().getJointDoFIndices(f.filteredJoints.get(i))[0];
      int[] baseCols = input.getJointMatrixIndexProvider().getJointDoFIndices(f.rootJoint);
      assertEquals(6, baseCols.length, "floating base is 6-DoF");
      List<Integer> nuisance = new ArrayList<>();
      for (int c : baseCols)
         nuisance.add(c);
      for (JointReadOnly spanningJoint : spanning)
         if (!f.filteredJoints.contains(spanningJoint)) // gap joint => marginalize
            nuisance.add(input.getJointMatrixIndexProvider().getJointDoFIndices(spanningJoint)[0]);
      int nN = nuisance.size();

      DMatrixRMaj mNN = new DMatrixRMaj(nN, nN);
      DMatrixRMaj mNf = new DMatrixRMaj(nN, n);
      DMatrixRMaj mff = new DMatrixRMaj(n, n);
      for (int a = 0; a < nN; a++)
      {
         for (int b = 0; b < nN; b++)
            mNN.set(a, b, massMatrix.get(nuisance.get(a), nuisance.get(b)));
         for (int j = 0; j < n; j++)
            mNf.set(a, j, massMatrix.get(nuisance.get(a), filteredCols[j]));
      }
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            mff.set(i, j, massMatrix.get(filteredCols[i], filteredCols[j]));

      DMatrixRMaj mNNInv = new DMatrixRMaj(nN, nN);
      assertTrue(CommonOps_DDRM.invert(mNN, mNNInv), "reference M_NN inverts");
      DMatrixRMaj x = new DMatrixRMaj(nN, n);
      CommonOps_DDRM.mult(mNNInv, mNf, x);       // X = M_NN⁻¹ M_Nf
      DMatrixRMaj mjNX = new DMatrixRMaj(n, n);
      CommonOps_DDRM.multTransA(mNf, x, mjNX);   // M_Nf^T X = M_jN X
      DMatrixRMaj lambda = new DMatrixRMaj(n, n);
      CommonOps_DDRM.subtract(mff, mjNX, lambda); // Λ = M_ff − M_jN X
      return new DMatrixRMaj[] {lambda, mff};
   }

   /** Independent reference: Qa = σ_τ² (Λ⁻¹)², symmetrized, in filter state order. */
   private static DMatrixRMaj referenceQa(JointLevelKFTestFixture f)
   {
      int n = f.n;
      DMatrixRMaj lambda = referenceSchur(f)[0];
      DMatrixRMaj lambdaInv = new DMatrixRMaj(n, n);
      assertTrue(CommonOps_DDRM.invert(lambda, lambdaInv), "reference Λ inverts");
      DMatrixRMaj lambdaInvSq = new DMatrixRMaj(n, n);
      CommonOps_DDRM.mult(lambdaInv, lambdaInv, lambdaInvSq);

      DMatrixRMaj qa = new DMatrixRMaj(n, n);
      double st2 = SIGMA_TAU * SIGMA_TAU;
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            qa.set(i, j, st2 * 0.5 * (lambdaInvSq.get(i, j) + lambdaInvSq.get(j, i)));
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
   public void testProcessNoiseEqualsVanLoanOfSchurComplementInverseSquared()
   {
      // Decisive oracle for the Schur algebra: the filter's Q joint blocks must equal the Van Loan discretization
      // of Qa = σ_τ² Λ⁻², with Λ recomputed by the fully independent reference (§3.2). Any block-extraction or
      // sign error in M_bb / M_bj / M_jj shows up here.
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(6100L))
      {
         int n = f.n;
         DMatrixRMaj qa = referenceQa(f);
         DMatrixRMaj Q = f.filter.getProcessNoise();

         DMatrixRMaj expected = new DMatrixRMaj(n, n);
         CommonOps_DDRM.scale(DT * DT * DT / 3.0, qa, expected);
         assertAllClose(block(Q, 0, 0, n, n), expected, relTol(expected), f.describe() + " Q qq = dt³/3 σ_τ² Λ⁻²");
         CommonOps_DDRM.scale(DT * DT / 2.0, qa, expected);
         assertAllClose(block(Q, 0, n, n, n), expected, relTol(expected), f.describe() + " Q q,q̇ = dt²/2 σ_τ² Λ⁻²");
         assertAllClose(block(Q, n, 0, n, n), expected, relTol(expected), f.describe() + " Q q̇,q = dt²/2 σ_τ² Λ⁻²");
         CommonOps_DDRM.scale(DT, qa, expected);
         assertAllClose(block(Q, n, n, n, n), expected, relTol(expected), f.describe() + " Q q̇q̇ = dt σ_τ² Λ⁻²");
      }
   }

   @Test
   public void testSchurComplementIsSymmetricPDAndDominatedByLockedInertia()
   {
      // SPEC §3.2/§8: Λ is symmetric PD whenever M is, and Λ ⪯ M_ff (the free base/nuisance recoils, so the
      // joints accelerate MORE per unit torque than the fully-locked model — hence Λ⁻² ⪰ M_ff⁻² and the
      // effective Qa grows). M_ff is the filtered block with everything else locked; for the real gapless
      // topology M_ff = the locked-base joint inertia M_jj. Assert Λ symmetric PD and the PSD ordering
      // M_ff − Λ ⪰ 0, over the shape spread and at a strongly-bent configuration (non-trivial coupling M_jN).
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(6800L))
      {
         int n = f.n;
         double[] q = new double[n];
         double[] qd = new double[n];
         for (int i = 0; i < n; i++)
            q[i] = 0.7 * ((i % 2 == 0) ? 1.0 : -1.0);
         f.applyConsistentMotion(q, qd);

         DMatrixRMaj[] schur = referenceSchur(f);
         DMatrixRMaj lambda = schur[0];
         DMatrixRMaj mff = schur[1];

         assertSymmetric(lambda, 1.0e-9 * Math.max(1.0, CommonOps_DDRM.elementMaxAbs(lambda)), f.describe() + " Λ symmetric");
         assertPositiveSemiDefinite(lambda, f.describe() + " Λ PD");

         DMatrixRMaj mffMinusLambda = new DMatrixRMaj(n, n);
         CommonOps_DDRM.subtract(mff, lambda, mffMinusLambda); // M_ff − Λ = M_jN M_NN⁻¹ M_Nj ⪰ 0
         assertPositiveSemiDefinite(mffMinusLambda, f.describe() + " M_ff − Λ ⪰ 0 (Λ ⪯ M_ff)");
      }
   }

   @Test
   public void testVanLoanBlocksAreExactlySymmetric()
   {
      // The symmetrized read of Λ⁻² (SPEC §3.4) must make every joint block of Q exactly symmetric and the two
      // q–q̇ cross blocks exactly equal — the Joseph update depends on it. "Exactly" here means to the last bit
      // (0.0 tolerance): the filter symmetrizes with 0.5·(A+Aᵀ), which is bit-symmetric by construction.
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(6900L))
      {
         int n = f.n;
         DMatrixRMaj Q = f.filter.getProcessNoise();
         DMatrixRMaj qq = block(Q, 0, 0, n, n);
         DMatrixRMaj qdqd = block(Q, n, n, n, n);
         DMatrixRMaj qqd = block(Q, 0, n, n, n);
         DMatrixRMaj qdq = block(Q, n, 0, n, n);
         for (int i = 0; i < n; i++)
         {
            for (int j = 0; j < n; j++)
            {
               assertEquals(qq.get(i, j), qq.get(j, i), 0.0, f.describe() + " Q qq exactly symmetric");
               assertEquals(qdqd.get(i, j), qdqd.get(j, i), 0.0, f.describe() + " Q q̇q̇ exactly symmetric");
               assertEquals(qqd.get(i, j), qdq.get(j, i), 0.0, f.describe() + " Q q,q̇ block equals (q̇,q)ᵀ exactly");
            }
         }
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
