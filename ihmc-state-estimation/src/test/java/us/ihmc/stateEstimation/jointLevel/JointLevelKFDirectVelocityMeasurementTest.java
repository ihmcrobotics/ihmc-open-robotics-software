package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Direct joint-velocity measurement channel: wiring, adaptive lag inflation, and chi-square NIS consistency.
 *
 * <p><b>Lag-inflation property.</b> The measured q̇ is the output of a first-order low-pass cascade with
 * effective corner ω_eff; the identity {@code u − y = ẏ/ω} makes the instantaneous lag error EQUAL to the
 * measured signal's slope over the corner. The channel therefore applies
 * {@code R_ii(t) = σ_i² + (d̂_i/ω_eff,i)²} with d̂ a 5 Hz-smoothed finite difference of the measurement. The
 * property tested: a CONSTANT measurement relaxes R to the measured floor σ² (full sharpness at quiet stance),
 * and a RAMP of slope s inflates R to exactly {@code σ² + (s/ω_eff)²} once the smoother settles
 * (d̂ → s·(1−α^k), α = exp(−2π·5·dt), so 300 ticks at dt = 1 ms leaves a relative error below 1e-4).</p>
 *
 * <p><b>NIS property.</b> Identical construction to {@link JointLevelKFEncoderNISConsistencyTest}, on the q̇
 * rows: truth sampled from the filter's own prior makes per-row NIS exactly χ²₁ when the injected noise
 * matches the wired R, and a 3σ injection must blow the mean past 4 (expected 7.4 with P = R/4).</p>
 */
class JointLevelKFDirectVelocityMeasurementTest
{
   /** Mirrors the private JointLevelKFPreFilter.SIGMA_QD_UNFILTERED fallback (rad/s). */
   private static final double SIGMA_QD_FALLBACK = 0.1;
   private static final double DT = JointLevelKFTestFixture.DT;

   private static final long SEED = 47_001L;
   private static final int NUM_CHAIN_JOINTS = 10; // singlePair(1, 9) => n = 8 filtered joints

   private static double posSigmaFor(String jointName)
   {
      return 1.0e-4 * (1 + Math.floorMod(jointName.hashCode(), 9));
   }

   /** Deterministic per-joint velocity noise STD in [5e-3, 3.5e-2] rad/s — the measured-table range. */
   private static double velSigmaFor(String jointName)
   {
      return 5.0e-3 * (1 + Math.floorMod(jointName.hashCode(), 7));
   }

   @Test
   void velocityMeasurementModelIsWired()
   {
      JointLevelKFTestFixture fixture = JointLevelKFTestFixture.singlePairWithDirectVelocity(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                             JointLevelKFDirectVelocityMeasurementTest::posSigmaFor,
                                                                                             JointLevelKFDirectVelocityMeasurementTest::velSigmaFor,
                                                                                             null);
      int n = fixture.n;
      int dim = fixture.dim;
      DMatrixRMaj Hqd = fixture.filter.getVelocityMeasurementJacobian();
      assertEquals(n, Hqd.getNumRows());
      assertEquals(dim, Hqd.getNumCols());
      for (int i = 0; i < n; i++)
         for (int j = 0; j < dim; j++)
            assertEquals(j == n + i ? 1.0 : 0.0, Hqd.get(i, j), 0.0, "Hqd must be [0 | I_n | 0]");

      DMatrixRMaj Rqd = fixture.filter.getVelocityMeasurementNoise();
      for (int i = 0; i < n; i++)
      {
         String name = fixture.filteredJoints.get(i).getName();
         double expectedVar = velSigmaFor(name) * velSigmaFor(name);
         assertEquals(expectedVar, Rqd.get(i, i), 1.0e-15, "Rqd diagonal for " + name);
         YoDouble yoQdR = (YoDouble) fixture.registry.findVariable("jointKF_qdR_" + name);
         assertNotNull(yoQdR, "jointKF_qdR_" + name + " must be published");
         assertEquals(expectedVar, yoQdR.getValue(), 1.0e-15, "published velocity variance for " + name);
      }
   }

   @Test
   void unmatchedJointFallsBackToSigmaQdUnfiltered()
   {
      JointLevelKFTestFixture probe = JointLevelKFTestFixture.singlePairWithDirectVelocity(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                           JointLevelKFDirectVelocityMeasurementTest::posSigmaFor,
                                                                                           JointLevelKFDirectVelocityMeasurementTest::velSigmaFor,
                                                                                           null);
      String unmatched = probe.filteredJoints.get(0).getName();
      JointLevelKFTestFixture fixture = JointLevelKFTestFixture.singlePairWithDirectVelocity(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                             JointLevelKFDirectVelocityMeasurementTest::posSigmaFor,
                                                                                             name -> name.equals(unmatched) ? Double.NaN : velSigmaFor(name),
                                                                                             null);
      DMatrixRMaj Rqd = fixture.filter.getVelocityMeasurementNoise();
      for (int i = 0; i < fixture.n; i++)
      {
         String name = fixture.filteredJoints.get(i).getName();
         double expectedVar = name.equals(unmatched) ? SIGMA_QD_FALLBACK * SIGMA_QD_FALLBACK : velSigmaFor(name) * velSigmaFor(name);
         assertEquals(expectedVar, Rqd.get(i, i), 1.0e-15, "Rqd diagonal for " + name);
      }
   }

   @Test
   void lagInflationTracksMeasuredSlewExactly()
   {
      double cornerHz = 10.0;
      double invOmega = 1.0 / (2.0 * Math.PI * cornerHz);
      JointLevelKFTestFixture fixture = JointLevelKFTestFixture.singlePairWithDirectVelocity(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                             JointLevelKFDirectVelocityMeasurementTest::posSigmaFor,
                                                                                             JointLevelKFDirectVelocityMeasurementTest::velSigmaFor,
                                                                                             name -> cornerHz);
      int n = fixture.n;
      double[] z = new double[n];

      // Phase 1 — constant measurement (quiet stance): the slew estimate stays 0 and R sits at the floor.
      for (int t = 0; t < 100; t++)
      {
         fixture.filter.setDirectVelocityMeasurementForTest(z);
         fixture.filter.refreshDirectVelocityNoise();
      }
      DMatrixRMaj Rquiet = fixture.filter.getVelocityMeasurementNoise();
      for (int i = 0; i < n; i++)
      {
         double sigma2 = Math.pow(velSigmaFor(fixture.filteredJoints.get(i).getName()), 2);
         assertEquals(sigma2, Rquiet.get(i, i), 1.0e-12, "quiet stance must run at the measured floor");
      }

      // Phase 2 — ramp of slope s (a swing): d̂ settles to s and R inflates by exactly (s/ω_eff)².
      double slope = 2.0; // rad/s per s
      for (int t = 1; t <= 300; t++)
      {
         for (int i = 0; i < n; i++)
            z[i] = slope * t * DT;
         fixture.filter.setDirectVelocityMeasurementForTest(z);
         fixture.filter.refreshDirectVelocityNoise();
      }
      DMatrixRMaj Rswing = fixture.filter.getVelocityMeasurementNoise();
      double lagVar = Math.pow(slope * invOmega, 2);
      for (int i = 0; i < n; i++)
      {
         String name = fixture.filteredJoints.get(i).getName();
         double expected = Math.pow(velSigmaFor(name), 2) + lagVar;
         assertEquals(expected, Rswing.get(i, i), 1.0e-3 * expected, "swing must inflate by the first-order lag error for " + name);
         YoDouble yoQdR = (YoDouble) fixture.registry.findVariable("jointKF_qdR_" + name);
         assertEquals(Rswing.get(i, i), yoQdR.getValue(), 0.0, "published R must be the applied R");
      }

      // Phase 3 — back to constant: the inflation must decay again (no latch-up).
      for (int t = 0; t < 2000; t++)
      {
         fixture.filter.setDirectVelocityMeasurementForTest(z); // z frozen at its last ramp value
         fixture.filter.refreshDirectVelocityNoise();
      }
      DMatrixRMaj Rsettled = fixture.filter.getVelocityMeasurementNoise();
      for (int i = 0; i < n; i++)
      {
         double sigma2 = Math.pow(velSigmaFor(fixture.filteredJoints.get(i).getName()), 2);
         assertTrue(Rsettled.get(i, i) < sigma2 * 1.01, "inflation must decay once the motion stops");
      }
   }

   @Test
   void directVelocityNISIsChiSquareConsistentAtTheWiredNoise()
   {
      double[] meanNIS = runNISTrials(1.0, 4000);
      double envelope = 4.0 * Math.sqrt(2.0 / 4000);
      for (int i = 0; i < meanNIS.length; i++)
         assertTrue(Math.abs(meanNIS[i] - 1.0) < envelope,
                    "joint " + i + ": qd NIS mean " + meanNIS[i] + " outside 1 +/- " + envelope);
   }

   @Test
   void directVelocityNISCatchesUnderstatedR()
   {
      double[] meanNIS = runNISTrials(3.0, 4000);
      for (int i = 0; i < meanNIS.length; i++)
         assertTrue(meanNIS[i] > 4.0, "joint " + i + ": qd NIS mean " + meanNIS[i] + " should expose the 9x-understated R (expected ~7.4)");
   }

   private static double[] runNISTrials(double noiseScale, int trials)
   {
      JointLevelKFTestFixture fixture = JointLevelKFTestFixture.singlePairWithDirectVelocity(SEED, NUM_CHAIN_JOINTS, 1, 9,
                                                                                             JointLevelKFDirectVelocityMeasurementTest::posSigmaFor,
                                                                                             JointLevelKFDirectVelocityMeasurementTest::velSigmaFor,
                                                                                             null); // no corner: static R, so S is deterministic
      int n = fixture.n;
      int dim = fixture.dim;
      DMatrixRMaj Hqd = fixture.filter.getVelocityMeasurementJacobian();
      DMatrixRMaj Rqd = fixture.filter.getVelocityMeasurementNoise();

      DMatrixRMaj xPrior = new DMatrixRMaj(dim, 1);
      DMatrixRMaj pPrior = new DMatrixRMaj(dim, dim);
      for (int i = 0; i < n; i++)
         pPrior.set(i, i, 1.0e-4);
      for (int i = 0; i < n; i++)
         pPrior.set(n + i, n + i, 0.25 * Rqd.get(i, i));
      for (int i = 2 * n; i < dim; i++)
         pPrior.set(i, i, 1.0e-6);

      YoDouble[] yoNIS = new YoDouble[n];
      YoDouble[] yoEncNIS = new YoDouble[n];
      for (int i = 0; i < n; i++)
      {
         String name = fixture.filteredJoints.get(i).getName();
         yoNIS[i] = (YoDouble) fixture.registry.findVariable("jointKF_qdNIS_" + name);
         yoEncNIS[i] = (YoDouble) fixture.registry.findVariable("jointKF_encNIS_" + name);
         assertNotNull(yoNIS[i]);
      }

      Random random = new Random(SEED + 13);
      DMatrixRMaj z = new DMatrixRMaj(n, 1);
      double[] sumNIS = new double[n];
      for (int t = 0; t < trials; t++)
      {
         fixture.filter.setStateForTest(xPrior, pPrior);
         for (int i = 0; i < n; i++)
         {
            double truth = xPrior.get(n + i, 0) + Math.sqrt(pPrior.get(n + i, n + i)) * random.nextGaussian();
            double noise = noiseScale * Math.sqrt(Rqd.get(i, i)) * random.nextGaussian();
            z.set(i, 0, truth + noise);
         }
         fixture.filter.josephUpdate(Hqd, z, Rqd, "encoderVelocity");
         for (int i = 0; i < n; i++)
         {
            double nis = yoNIS[i].getValue();
            assertTrue(Double.isFinite(nis) && nis >= 0.0, "qd NIS must be finite and non-negative");
            sumNIS[i] += nis;
         }
      }

      // Cross-talk guard: an "encoderVelocity" update must never publish into the position NIS (regression
      // for the startsWith("encoder") dispatch this channel forced to become an exact match).
      for (int i = 0; i < n; i++)
         assertTrue(Double.isNaN(yoEncNIS[i].getValue()), "encoder NIS must stay untouched by the velocity channel");

      double[] mean = new double[n];
      for (int i = 0; i < n; i++)
         mean[i] = sumNIS[i] / trials;
      return mean;
   }
}
