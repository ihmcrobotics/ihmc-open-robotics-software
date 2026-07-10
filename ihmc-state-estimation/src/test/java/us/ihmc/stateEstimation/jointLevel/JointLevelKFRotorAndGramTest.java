package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.junit.jupiter.api.Test;

/**
 * Machine-precision oracle for the two algebraic halves of the Part B process-noise fix, on a WELL-conditioned
 * synthetic {@code Λ} so no ill-conditioning masks a sign/index error (the random-fixture oracle in
 * {@link JointLevelKFMassMatrixNoiseTest} runs the same algebra through the live filter but can only assert to
 * ~1e-3 because LU vs Cholesky diverge on nearly-singular random floating-base blocks — see its {@code relTol}).
 *
 * <ul>
 *   <li><b>Item 1 — reflected rotor inertia.</b> {@code Lambda_eff = Λ + diag(J_rotor)} floors the spectrum by
 *   Weyl: {@code λ_min(Lambda_eff) ≥ λ_min(Λ) + min_i J_rotor,i}. Verified against a dense eigendecomposition.</li>
 *   <li><b>Item 3/6 — per-joint σ_τ via the Gram form.</b> {@code Qa = Y Yᵀ}, {@code Y = Lambda_eff⁻¹ diag(σ_τ)}
 *   equals {@code Lambda_eff⁻¹ diag(σ_τ²) Lambda_eff⁻ᵀ} exactly, is exactly symmetric, and is PSD.</li>
 * </ul>
 *
 * <p>This mirrors the exact arithmetic of {@code updateProcessNoiseFromMassMatrix} (rotor diagonal add, then
 * {@code multTransB(Y, Y)}). It uses {@link JointLevelKFPreFilter#reflectedRotorInertiaForNameOrDefault} so the
 * rotor-table lookup itself is covered.</p>
 */
public class JointLevelKFRotorAndGramTest
{
   /** Item 1: Lambda_eff = Λ + diag(rotor) and the Weyl λ_min floor. */
   @Test
   public void testReflectedRotorInertiaAddAndWeylFloor()
   {
      // A well-conditioned, deliberately light-diagonal Λ (a "near-singular proximal joint" at index 1).
      DMatrixRMaj lambda = new DMatrixRMaj(new double[][] {
            {1.20, 0.10, 0.02},
            {0.10, 0.03, 0.01},   // light mode -> small λ_min without the rotor floor
            {0.02, 0.01, 0.90}});
      double[] rotor = {0.062, 0.167, 0.070}; // e.g. HIP_X, HIP_Y, ANKLE_Y table values

      DMatrixRMaj lambdaEff = lambda.copy();
      for (int i = 0; i < 3; i++)
         lambdaEff.add(i, i, rotor[i]);

      // Hand-computed diagonal add is exact.
      for (int i = 0; i < 3; i++)
         assertEquals(lambda.get(i, i) + rotor[i], lambdaEff.get(i, i), 0.0, "rotor diagonal add is exact at " + i);
      for (int i = 0; i < 3; i++)
         for (int j = 0; j < 3; j++)
            if (i != j)
               assertEquals(lambda.get(i, j), lambdaEff.get(i, j), 0.0, "rotor add touches only the diagonal");

      // Weyl: λ_min(Λ + D) ≥ λ_min(Λ) + λ_min(D) = λ_min(Λ) + min_i rotor_i.
      double minRotor = Math.min(rotor[0], Math.min(rotor[1], rotor[2]));
      double lminLambda = minEig(lambda);
      double lminEff = minEig(lambdaEff);
      assertTrue(lminEff >= lminLambda + minRotor - 1.0e-12,
                 "Weyl floor: λ_min(Lambda_eff)=" + lminEff + " ≥ λ_min(Λ)+min(rotor)=" + (lminLambda + minRotor));
      assertTrue(lminEff >= minRotor - 1.0e-12, "Lambda_eff spectrum is floored at least at min(rotor)=" + minRotor);
   }

   /** Item 3/6: Gram-form Qa is exactly symmetric PSD and equals the dense Lambda_eff⁻¹ diag(σ²) Lambda_eff⁻ᵀ. */
   @Test
   public void testGramFormQaEqualsDenseReferenceAndIsSymmetricPSD()
   {
      DMatrixRMaj lambdaEff = new DMatrixRMaj(new double[][] {
            {1.262, 0.10, 0.02},
            {0.10, 0.197, 0.01},
            {0.02, 0.01, 0.970}});
      double[] sigmaTau = {0.15 * 160.7, 0.15 * 217.2, 0.15 * 193.6}; // ALPHA * tau_max for HIP_X, HIP_Y, ANKLE_Y

      DMatrixRMaj lambdaInv = new DMatrixRMaj(3, 3);
      assertTrue(CommonOps_DDRM.invert(lambdaEff, lambdaInv), "Lambda_eff inverts");

      // Filter path: Y = Lambda_eff⁻¹ with column j scaled by σ_τ,j; Qa = Y Yᵀ.
      DMatrixRMaj y = new DMatrixRMaj(3, 3);
      for (int i = 0; i < 3; i++)
         for (int j = 0; j < 3; j++)
            y.set(i, j, lambdaInv.get(i, j) * sigmaTau[j]);
      DMatrixRMaj qaGram = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.multTransB(y, y, qaGram);

      // Dense reference: Lambda_eff⁻¹ diag(σ²) Lambda_eff⁻ᵀ.
      DMatrixRMaj diagSq = new DMatrixRMaj(3, 3);
      for (int i = 0; i < 3; i++)
         diagSq.set(i, i, sigmaTau[i] * sigmaTau[i]);
      DMatrixRMaj tmp = new DMatrixRMaj(3, 3);
      DMatrixRMaj qaRef = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(lambdaInv, diagSq, tmp);
      CommonOps_DDRM.multTransB(tmp, lambdaInv, qaRef); // lambdaInv symmetric, but use transpose form to match the algebra

      double tol = 1.0e-10 * Math.max(1.0, CommonOps_DDRM.elementMaxAbs(qaRef));
      for (int i = 0; i < 3; i++)
         for (int j = 0; j < 3; j++)
            assertEquals(qaRef.get(i, j), qaGram.get(i, j), tol, "Gram Qa == dense reference at (" + i + "," + j + ")");

      // Exactly symmetric by construction (Y Yᵀ).
      for (int i = 0; i < 3; i++)
         for (int j = i + 1; j < 3; j++)
            assertEquals(qaGram.get(i, j), qaGram.get(j, i), 0.0, "Qa exactly symmetric at (" + i + "," + j + ")");

      // PSD: every eigenvalue ≥ 0.
      assertTrue(minEig(qaGram) >= -1.0e-12, "Qa is PSD (min eig=" + minEig(qaGram) + ")");
   }

   /** The rotor-table lookup seam: known Alex joint-name substrings map to their table values; unmatched -> default. */
   @Test
   public void testRotorInertiaTableLookup()
   {
      assertEquals(0.062, JointLevelKFPreFilter.reflectedRotorInertiaForNameOrDefault("LEFT_HIP_X"), 0.0, "HIP_X");
      assertEquals(0.167, JointLevelKFPreFilter.reflectedRotorInertiaForNameOrDefault("RIGHT_HIP_Y"), 0.0, "HIP_Y");
      assertEquals(0.167, JointLevelKFPreFilter.reflectedRotorInertiaForNameOrDefault("left_knee_y"), 0.0, "KNEE (case-insensitive)");
      assertEquals(0.070, JointLevelKFPreFilter.reflectedRotorInertiaForNameOrDefault("LEFT_ANKLE_Y"), 0.0, "ANKLE_Y");
      assertEquals(0.050, JointLevelKFPreFilter.reflectedRotorInertiaForNameOrDefault("LEFT_ANKLE_X"), 0.0, "ANKLE_X");
      assertEquals(0.062, JointLevelKFPreFilter.reflectedRotorInertiaForNameOrDefault("SPINE_Z"), 0.0, "SPINE");
      assertEquals(0.005, JointLevelKFPreFilter.reflectedRotorInertiaForNameOrDefault("SOME_UNKNOWN_JOINT"), 0.0, "unmatched -> default floor");
   }

   private static double minEig(DMatrixRMaj a)
   {
      EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(a.numRows, false, true);
      assertTrue(eig.decompose(a.copy()), "eigendecomposition succeeds");
      double min = Double.POSITIVE_INFINITY;
      for (int i = 0; i < a.numRows; i++)
         min = Math.min(min, eig.getEigenvalue(i).getReal());
      return min;
   }
}
