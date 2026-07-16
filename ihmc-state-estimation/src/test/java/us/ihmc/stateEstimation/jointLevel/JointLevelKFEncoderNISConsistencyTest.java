package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Per-joint encoder measurement-noise wiring and chi-square NIS consistency.
 *
 * <p><b>Property.</b> For a linear KF update with prior {@code (x̂, P)} (P diagonal here) and encoder
 * measurement {@code z_i = q_true,i + e_i}, sample the truth from the filter's own prior,
 * {@code q_true,i = x̂_i + ε_i}, {@code ε_i ~ N(0, P_ii)}, and the noise at the wired variance,
 * {@code e_i ~ N(0, R_ii)}. Then the innovation is {@code ν_i = ε_i + e_i ~ N(0, P_ii + R_ii) = N(0, S_ii)}
 * exactly (H is the identity on the q block and P is diagonal, so S is diagonal), hence the per-joint
 * {@code NIS_i = ν_i² / S_ii ~ χ²₁} with mean 1 and variance 2. The sample mean over {@code M} independent
 * trials must land in {@code 1 ± 4·sqrt(2/M)} (4-sigma envelope, deterministic seed).</p>
 *
 * <p><b>Sensitivity.</b> The same statistic detects an understated R: drawing {@code e_i} at 3× the wired
 * sigma (9× the variance) moves the expected NIS mean to {@code (P_ii + 9 R_ii)/(P_ii + R_ii)} = 7.4 for the
 * {@code P_ii = R_ii/4} prior used here. This is exactly the mismatch class the per-joint wiring fixes: the
 * old scalar ENCODER_VAR sat orders of magnitude away from the measured per-joint values, and a hardware
 * {@code jointKF_encNIS_<joint>} mean far from 1 reads directly as "R is wrong by that factor".</p>
 */
class JointLevelKFEncoderNISConsistencyTest
{
   /** Mirrors the private JointLevelKFPreFilter.ENCODER_VAR scalar fallback (rad^2). */
   private static final double ENCODER_VAR_FALLBACK = 5.0e-5;

   private static final long SEED = 31_001L;
   private static final int NUM_CHAIN_JOINTS = 10; // singlePair(1, 9) => n = 8 filtered joints

   /** Deterministic, per-joint-distinct encoder noise STD in [1e-4, 9e-4] rad — spans ~1 order of magnitude in variance. */
   private static double sigmaFor(String jointName)
   {
      return 1.0e-4 * (1 + Math.floorMod(jointName.hashCode(), 9));
   }

   @Test
   void perJointEncoderVarianceIsWiredIntoREncAndPublished()
   {
      JointLevelKFTestFixture fixture = JointLevelKFTestFixture.singlePairWithEncoderNoise(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                           JointLevelKFEncoderNISConsistencyTest::sigmaFor);
      DMatrixRMaj Renc = fixture.filter.getEncoderNoise();
      assertEquals(fixture.n, Renc.getNumRows());
      for (int i = 0; i < fixture.n; i++)
      {
         String name = fixture.filteredJoints.get(i).getName();
         double expectedVar = sigmaFor(name) * sigmaFor(name);
         assertEquals(expectedVar, Renc.get(i, i), 1.0e-15, "Renc diagonal for " + name);
         for (int j = 0; j < fixture.n; j++)
            if (j != i)
               assertEquals(0.0, Renc.get(i, j), 0.0, "Renc must stay diagonal");

         YoDouble yoEncR = (YoDouble) fixture.registry.findVariable("jointKF_encR_" + name);
         assertNotNull(yoEncR, "jointKF_encR_" + name + " must be published");
         assertEquals(expectedVar, yoEncR.getValue(), 1.0e-15, "published wired variance for " + name);
      }
   }

   @Test
   void unmatchedJointFallsBackToScalarEncoderVar()
   {
      // Same seed twice => identical chain and joint names, so the first build tells us which name to NaN out.
      JointLevelKFTestFixture probe = JointLevelKFTestFixture.singlePairWithEncoderNoise(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                         JointLevelKFEncoderNISConsistencyTest::sigmaFor);
      String unmatched = probe.filteredJoints.get(0).getName();

      JointLevelKFTestFixture fixture = JointLevelKFTestFixture.singlePairWithEncoderNoise(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                           name -> name.equals(unmatched) ? Double.NaN : sigmaFor(name));
      DMatrixRMaj Renc = fixture.filter.getEncoderNoise();
      for (int i = 0; i < fixture.n; i++)
      {
         String name = fixture.filteredJoints.get(i).getName();
         double expectedVar = name.equals(unmatched) ? ENCODER_VAR_FALLBACK : sigmaFor(name) * sigmaFor(name);
         assertEquals(expectedVar, Renc.get(i, i), 1.0e-15, "Renc diagonal for " + name);
      }
   }

   @Test
   void encoderNISIsChiSquareConsistentAtTheWiredNoise()
   {
      double[] meanNIS = runNISTrials(1.0, 4000);
      double envelope = 4.0 * Math.sqrt(2.0 / 4000); // 4-sigma on the chi^2_1 sample mean ~ 0.089
      for (int i = 0; i < meanNIS.length; i++)
         assertTrue(Math.abs(meanNIS[i] - 1.0) < envelope,
                    "joint " + i + ": NIS mean " + meanNIS[i] + " outside 1 +/- " + envelope
                          + " — wired R inconsistent with the injected noise");
   }

   @Test
   void encoderNISCatchesUnderstatedR()
   {
      // Noise injected at 3x the wired sigma (9x variance): E[NIS] = (P + 9R)/(P + R) = 7.4 for P = R/4.
      // This is the pre-fix situation in miniature (measured variance far above the wired one) — the statistic
      // must move far from 1, or it could not have caught the original scalar-vs-measured mismatch.
      double[] meanNIS = runNISTrials(3.0, 4000);
      for (int i = 0; i < meanNIS.length; i++)
         assertTrue(meanNIS[i] > 4.0, "joint " + i + ": NIS mean " + meanNIS[i] + " should expose the 9x-understated R (expected ~7.4)");
   }

   /**
    * Runs {@code trials} independent single-update experiments and returns the per-joint NIS sample mean.
    * Each trial resets the filter to the same prior, samples the truth from that prior, injects encoder noise
    * at {@code noiseScale} times the wired sigma, applies the filter's own encoder model via
    * {@code josephUpdate(Henc, z, Renc, "encoder")}, and reads back {@code jointKF_encNIS_<joint>}.
    */
   private static double[] runNISTrials(double noiseScale, int trials)
   {
      JointLevelKFTestFixture fixture = JointLevelKFTestFixture.singlePairWithEncoderNoise(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                           JointLevelKFEncoderNISConsistencyTest::sigmaFor);
      int n = fixture.n;
      int dim = fixture.dim;
      DMatrixRMaj Henc = fixture.filter.getEncoderJacobian();
      DMatrixRMaj Renc = fixture.filter.getEncoderNoise();

      // Diagonal prior: q at a quarter of the wired variance (so an R error dominates S and the sensitivity
      // case moves the mean by ~R_true/R_wired), qd and bias at benign PD values.
      DMatrixRMaj xPrior = new DMatrixRMaj(dim, 1);
      DMatrixRMaj pPrior = new DMatrixRMaj(dim, dim);
      for (int i = 0; i < n; i++)
         pPrior.set(i, i, 0.25 * Renc.get(i, i));
      for (int i = n; i < 2 * n; i++)
         pPrior.set(i, i, 1.0e-2);
      for (int i = 2 * n; i < dim; i++)
         pPrior.set(i, i, 1.0e-6);

      YoDouble[] yoNIS = new YoDouble[n];
      for (int i = 0; i < n; i++)
      {
         yoNIS[i] = (YoDouble) fixture.registry.findVariable("jointKF_encNIS_" + fixture.filteredJoints.get(i).getName());
         assertNotNull(yoNIS[i]);
      }

      Random random = new Random(SEED + 7);
      DMatrixRMaj z = new DMatrixRMaj(n, 1);
      double[] sumNIS = new double[n];
      for (int t = 0; t < trials; t++)
      {
         fixture.filter.setStateForTest(xPrior, pPrior); // identical prior every trial => S is deterministic
         for (int i = 0; i < n; i++)
         {
            double truth = xPrior.get(i, 0) + Math.sqrt(pPrior.get(i, i)) * random.nextGaussian();
            double noise = noiseScale * Math.sqrt(Renc.get(i, i)) * random.nextGaussian();
            z.set(i, 0, truth + noise);
         }
         fixture.filter.josephUpdate(Henc, z, Renc, "encoder");
         for (int i = 0; i < n; i++)
         {
            double nis = yoNIS[i].getValue();
            assertTrue(Double.isFinite(nis) && nis >= 0.0, "NIS must be finite and non-negative");
            sumNIS[i] += nis;
         }
         // Signed innovation publishes: nu_i = z_i - (H x^-)_i = z_i - xPrior_i (H is identity on the q block).
         if (t == 0)
            for (int i = 0; i < n; i++)
               assertEquals(z.get(i, 0) - xPrior.get(i, 0),
                            ((YoDouble) fixture.registry.findVariable("jointKF_encInnov_" + fixture.filteredJoints.get(i).getName())).getValue(),
                            1.0e-15, "signed encoder innovation must equal z - Hx");
      }

      double[] mean = new double[n];
      for (int i = 0; i < n; i++)
         mean[i] = sumNIS[i] / trials;
      return mean;
   }
}
