package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertTrue;

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
 * Reconciliation harness for the Alex002 hardware-log finding: the joint VELOCITY covariance blows up in ONE
 * predict() while the joint POSITION covariance stays sane, driven by the Schur process noise Qa = σ_τ² Λ⁻².
 * These tests quantify Λ's conditioning, confirm predict()/Qa (not the gyro update) is the inflation source,
 * rank the candidate fixes at the matrix level, and provide the property test the fix must pass.
 */
public class JointLevelKFStandingStabilityTest
{
   private static final double DT = JointLevelKFTestFixture.DT;
   private static final double SIGMA_TAU = 5.0; // matches the (fixed) filter; CHECK #3 uses explicit 50/5/1 literals
   // Physical cap on the joint-acceleration process-noise variance (σ_qdd_max = 30 rad/s²)² — mirrors the filter's
   // JointLevelKFPreFilter.QA_MAX after the fix, used to validate the conditioning cap independently.
   private static final double QA_MAX = 900.0;

   /** Λ = M_ff − M_jN M_NN⁻¹ M_Nf and M_ff, in filter state order — independent EJML/LU reference (copied from
    *  JointLevelKFMassMatrixNoiseTest.referenceSchur so this test stands alone). */
   private static DMatrixRMaj[] referenceSchur(JointLevelKFTestFixture f)
   {
      int n = f.n;
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
      consider.add(f.rootJoint);
      consider.addAll(spanning);
      MultiBodySystemReadOnly input = MultiBodySystemReadOnly.toMultiBodySystemInput(consider);
      CompositeRigidBodyMassMatrixCalculator calc = new CompositeRigidBodyMassMatrixCalculator(input);
      calc.reset();
      DMatrixRMaj massMatrix = calc.getMassMatrix().copy();

      int[] filteredCols = new int[n];
      for (int i = 0; i < n; i++)
         filteredCols[i] = input.getJointMatrixIndexProvider().getJointDoFIndices(f.filteredJoints.get(i))[0];
      int[] baseCols = input.getJointMatrixIndexProvider().getJointDoFIndices(f.rootJoint);
      List<Integer> nuisance = new ArrayList<>();
      for (int c : baseCols)
         nuisance.add(c);
      for (JointReadOnly spanningJoint : spanning)
         if (!f.filteredJoints.contains(spanningJoint))
            nuisance.add(input.getJointMatrixIndexProvider().getJointDoFIndices(spanningJoint)[0]);
      int nN = nuisance.size();

      DMatrixRMaj mNN = new DMatrixRMaj(nN, nN), mNf = new DMatrixRMaj(nN, n), mff = new DMatrixRMaj(n, n);
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
      CommonOps_DDRM.invert(mNN, mNNInv);
      DMatrixRMaj x = new DMatrixRMaj(nN, n);
      CommonOps_DDRM.mult(mNNInv, mNf, x);
      DMatrixRMaj mjNX = new DMatrixRMaj(n, n);
      CommonOps_DDRM.multTransA(mNf, x, mjNX);
      DMatrixRMaj lambda = new DMatrixRMaj(n, n);
      CommonOps_DDRM.subtract(mff, mjNX, lambda);
      return new DMatrixRMaj[] {lambda, mff};
   }

   private static double condSPD(DMatrixRMaj a)
   {
      double[] e = JointLevelKFTestFixture.symmetricEigenvalues(a);
      double min = Double.POSITIVE_INFINITY, max = 0.0;
      for (double v : e) { min = Math.min(min, v); max = Math.max(max, v); }
      return max / min;
   }

   private static double minEig(DMatrixRMaj a)
   {
      double[] e = JointLevelKFTestFixture.symmetricEigenvalues(a);
      double min = Double.POSITIVE_INFINITY;
      for (double v : e) min = Math.min(min, v);
      return min;
   }

   /** Qa = σ_τ² Λ⁻² given Λ. */
   private static DMatrixRMaj qaFromLambda(DMatrixRMaj lambda, double sigmaTau)
   {
      int n = lambda.numRows;
      DMatrixRMaj inv = new DMatrixRMaj(n, n);
      CommonOps_DDRM.invert(lambda, inv);
      DMatrixRMaj sq = new DMatrixRMaj(n, n);
      CommonOps_DDRM.mult(inv, inv, sq);
      CommonOps_DDRM.scale(sigmaTau * sigmaTau, sq);
      return sq;
   }

   private static double maxDiag(DMatrixRMaj a)
   {
      double m = 0.0;
      for (int i = 0; i < a.numRows; i++) m = Math.max(m, Math.abs(a.get(i, i)));
      return m;
   }

   /** Qa = σ_τ² Λ⁻² with the physical conditioning cap: uniformly scale Qa down so no diagonal exceeds QA_MAX
    *  (preserves symmetry / PSD / joint coupling). Mirrors the filter's post-fix updateProcessNoiseFromMassMatrix. */
   private static DMatrixRMaj qaFromLambdaCapped(DMatrixRMaj lambda, double sigmaTau)
   {
      DMatrixRMaj qa = qaFromLambda(lambda, sigmaTau);
      double md = maxDiag(qa);
      if (md > QA_MAX)
         CommonOps_DDRM.scale(QA_MAX / md, qa);
      return qa;
   }

   // ============================ CHECK #1 + #2: Λ conditioning and predict-Qa mechanism ============================

   @Test
   public void testSchurConditioningAndPredictInflatesVelocityCovariance()
   {
      System.out.println("=== CHECK #1/#2: Λ conditioning + one-predict P_qdqd inflation (σ_τ=" + SIGMA_TAU + ") ===");
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(9100L))
      {
         int n = f.n;
         DMatrixRMaj lambda = referenceSchur(f)[0];
         DMatrixRMaj qa = qaFromLambdaCapped(lambda, SIGMA_TAU); // matches the fixed filter (σ_τ retune + cap)

         double cond = condSPD(lambda);
         double lmin = minEig(lambda);
         System.out.printf("%s  cond(Λ)=%.3e  λ_min(Λ)=%.3e  max diag(Qa)=%.3e%n", f.describe(), cond, lmin, maxDiag(qa));
         int worst = 0;
         for (int i = 1; i < n; i++) if (qa.get(i, i) > qa.get(worst, worst)) worst = i;
         System.out.printf("      worst joint idx=%d  Qa_ii=%.3e  Q_qdqd(one step)=dt*Qa_ii=%.3e%n",
                           worst, qa.get(worst, worst), DT * qa.get(worst, worst));

         // Seed a SANE standing covariance and run ONE predict(); the q̇q̇ block must jump by exactly the Van
         // Loan term dt*Qa (this is what predict ADDS), proving predict/Qa — not a gyro-update gain — is the source.
         DMatrixRMaj xPrior = new DMatrixRMaj(f.dim, 1);
         DMatrixRMaj pPrior = CommonOps_DDRM.identity(f.dim);
         CommonOps_DDRM.scale(1.0e-4, pPrior); // sane: ~1e-4 on every diagonal (positions AND velocities)
         f.filter.setStateForTest(xPrior, pPrior);
         DMatrixRMaj pBefore = f.filter.getCovariance();
         f.filter.predict();
         DMatrixRMaj pAfter = f.filter.getCovariance();

         double pqdBefore = pBefore.get(n + worst, n + worst);
         double pqdAfter = pAfter.get(n + worst, n + worst);
         double pqBefore = pBefore.get(worst, worst);
         double pqAfter = pAfter.get(worst, worst);
         System.out.printf("      worst joint: P_qdqd %.3e -> %.3e (x%.1f)   P_qq %.3e -> %.3e (x%.1f)%n",
                           pqdBefore, pqdAfter, pqdAfter / pqdBefore, pqBefore, pqAfter, pqAfter / pqBefore);

         // predict adds Q; the q̇q̇ increment equals dt*Qa (Van Loan). Assert that identity holds (mechanism proof).
         double expectedIncrement = DT * qa.get(worst, worst);
         double actualIncrement = pqdAfter - pqdBefore;
         assertTrue(Math.abs(actualIncrement - expectedIncrement) <= 1e-6 * Math.max(1.0, expectedIncrement),
                    f.describe() + " one-predict P_qdqd increment must equal dt*Qa (predict is the inflation source): "
                    + actualIncrement + " vs " + expectedIncrement);
      }
   }

   // ============================ CHECK #3: rank the three candidate fixes at the matrix level ============================

   @Test
   public void testCandidateFixesBoundQa()
   {
      System.out.println("=== CHECK #3: candidate-fix max diag(Qa) at each config (smaller = better bounded) ===");
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(9200L))
      {
         DMatrixRMaj[] schur = referenceSchur(f);
         DMatrixRMaj lambda = schur[0];
         DMatrixRMaj mff = schur[1];
         int n = lambda.numRows;

         double curr = maxDiag(qaFromLambda(lambda, 50.0));                 // current
         double sig5 = maxDiag(qaFromLambda(lambda, 5.0));                  // (a) σ_τ=5
         double sig1 = maxDiag(qaFromLambda(lambda, 1.0));                  // (a) σ_τ=1

         // (b) eigenvalue floor: clamp Λ's spectrum from below at a physical floor, keep σ_τ=50.
         DMatrixRMaj lambdaFloored = floorSpectrum(lambda, 0.05); // floor 0.05 kg m^2 (a sane min effective inertia)
         double floored = maxDiag(qaFromLambda(lambdaFloored, 50.0));

         // (c) locked-base Rev.1 map: Qa = σ_τ² M_ff⁻² (better conditioned since M_ff ⪰ Λ).
         double locked = maxDiag(qaFromLambda(mff, 50.0));

         System.out.printf("%s  n=%d  Qa_maxdiag: current(σ50)=%.3e  σ5=%.3e  σ1=%.3e  Λ-floor(σ50)=%.3e  lockedBase M_ff⁻²(σ50)=%.3e%n",
                           f.describe(), n, curr, sig5, sig1, floored, locked);

         // Locked-base must be no larger than the Schur Qa (Λ ⪯ M_ff  ⇒  Λ⁻² ⪰ M_ff⁻²), a PSD-ordering sanity check.
         assertTrue(locked <= curr * (1.0 + 1e-6),
                    f.describe() + " locked-base Qa (M_ff⁻²) must be ≤ Schur Qa (Λ⁻²) by the PSD ordering; got "
                    + locked + " vs " + curr);
      }
   }

   /** Clamp a symmetric PD matrix's eigenvalues to be ≥ floor (physical min effective inertia). */
   private static DMatrixRMaj floorSpectrum(DMatrixRMaj a, double floor)
   {
      // Simple, robust: A_floored = A + max(0, floor - λ_min) * I brings the smallest eigenvalue up to `floor`
      // (shifts the whole spectrum) — a conservative, allocation-simple conditioning that never lowers inertia.
      double lmin = minEig(a);
      DMatrixRMaj out = a.copy();
      if (lmin < floor)
         for (int i = 0; i < out.numRows; i++)
            out.set(i, i, out.get(i, i) + (floor - lmin));
      return out;
   }

   // ============================ CHECK #4: property test the fix must pass (run on current filter) ============================

   // Physical bound on the velocity variance a SINGLE 1 ms predict() may inject. A joint accelerates at most
   // ~50 rad/s², so in one tick Δq̇ ≲ 0.05 rad/s and the injected variance is ≪ 1 (rad/s)²; 1.0 is already ~10×
   // generous. The injected variance is exactly the Van Loan term dt·Qa = dt·σ_τ²·(Λ⁻²)_ii, so this is a direct,
   // model-independent bound on the mass-matrix process noise — the quantity that blew up on Alex002.
   private static final double ONE_TICK_QDD_VARIANCE_BOUND = 1.0; // (rad/s)² injected per predict()

   @Test
   public void testSinglePredictVelocityVarianceInjectionIsPhysicallyBounded()
   {
      // PROPERTY (regression gate): seed a sane standing P, run ONE predict(), and require that no joint's
      // velocity variance grew by more than ONE_TICK_QDD_VARIANCE_BOUND, and that P stayed symmetric PSD.
      // FAILS on current code (σ_τ=50, uncapped Schur Qa) — the Alex002 mechanism; PASSES with the σ_τ retune +
      // Λ-conditioning cap. Uses the filter's REAL process noise via predict()/getCovariance().
      StringBuilder report = new StringBuilder("=== PROPERTY: one-predict velocity-variance injection ≤ "
                                               + ONE_TICK_QDD_VARIANCE_BOUND + " (rad/s)² ===\n");
      double worstInjectionOverall = 0.0;
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapesMassMatrix(9300L))
      {
         int n = f.n;
         DMatrixRMaj xPrior = new DMatrixRMaj(f.dim, 1);
         DMatrixRMaj pPrior = CommonOps_DDRM.identity(f.dim);
         CommonOps_DDRM.scale(1.0e-4, pPrior); // sane standing prior: 1e-4 on every diagonal
         f.filter.setStateForTest(xPrior, pPrior);

         DMatrixRMaj pBefore = f.filter.getCovariance();
         f.filter.predict();
         DMatrixRMaj pAfter = f.filter.getCovariance();

         double worstInjection = 0.0;
         for (int i = 0; i < n; i++)
            worstInjection = Math.max(worstInjection, pAfter.get(n + i, n + i) - pBefore.get(n + i, n + i));
         worstInjectionOverall = Math.max(worstInjectionOverall, worstInjection);
         JointLevelKFTestFixture.assertPositiveSemiDefinite(pAfter, f.describe() + " P after predict PSD");
         JointLevelKFTestFixture.assertSymmetric(pAfter, 1e-9 * maxAbs(pAfter) + 1e-12, f.describe() + " P symmetric");
         report.append(String.format("%s  worst q̇-variance injected by one predict = %.3e (rad/s)²  -> %s%n",
                                      f.describe(), worstInjection,
                                      worstInjection <= ONE_TICK_QDD_VARIANCE_BOUND ? "PASS" : "FAIL"));
      }
      System.out.print(report);
      assertTrue(worstInjectionOverall <= ONE_TICK_QDD_VARIANCE_BOUND,
                 "a single predict() injected " + worstInjectionOverall + " (rad/s)² of joint-velocity variance "
                 + "(bound " + ONE_TICK_QDD_VARIANCE_BOUND + "): the un-retuned/uncapped Schur Qa = σ_τ²Λ⁻² is the "
                 + "Alex002 divergence mechanism.\n" + report);
   }

   @Test
   public void testConditioningCapBoundsQaEvenForNearSingularLambda()
   {
      // Independent validation of the STRUCTURAL half of the fix (the Λ-conditioning cap), which σ_τ retune alone
      // cannot provide: a deliberately near-singular Λ (λ_min ~ 1e-2, Alex's proximal-hip regime) must still yield
      // a physically-bounded Qa. Computes Qa the same way the filter does, with and without the cap.
      double[] nearSingularLambdaDiag = {2.0, 0.02, 1.5}; // one near-singular (proximal-hip-like) mode
      DMatrixRMaj lambda = CommonOps_DDRM.identity(3);
      for (int i = 0; i < 3; i++) lambda.set(i, i, nearSingularLambdaDiag[i]);

      DMatrixRMaj qaUncapped = qaFromLambda(lambda, 5.0);
      DMatrixRMaj qaCapped = qaFromLambdaCapped(lambda, 5.0);
      System.out.printf("=== cap validation (near-singular Λ, λ_min=%.2e): Qa maxdiag uncapped=%.3e capped=%.3e ===%n",
                        0.02, maxDiag(qaUncapped), maxDiag(qaCapped));
      assertTrue(maxDiag(qaUncapped) > QA_MAX * 1.5, "near-singular Λ makes uncapped Qa exceed the physical cap");
      assertTrue(maxDiag(qaCapped) <= QA_MAX * (1.0 + 1e-9), "the cap bounds Qa at the physical maximum");
   }

   private static double maxAbs(DMatrixRMaj a)
   {
      double m = 0.0;
      for (int i = 0; i < a.getNumElements(); i++) m = Math.max(m, Math.abs(a.get(i)));
      return m;
   }
}
