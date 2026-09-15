package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

/**
 * Ported from {@code tests/jointKF/test_state.py}. Locks in the bias-augmented state layout
 * {@code x = [q ; q_dot ; b_omega] ∈ R^{2n+3m}}, the initial mean seeding from the encoders, and the
 * initial covariance's block structure / prior confidences.
 *
 * <p>The reference's per-pair bias becomes per-IMU here (m = distinct IMUs), and the "bias tighter than both
 * joint priors" invariant is adapted to this filter's actual prior ordering: position (encoders trusted) &lt;
 * bias &lt; velocity (unknown at init).</p>
 */
public class JointLevelKFStateTest
{
   private static final double INIT_POS_VAR = 1.0e-6;
   private static final double INIT_VEL_VAR = 1.0;
   private static final double INIT_BIAS_VAR = 2.5e-3;

   @Test
   public void testInitStateShapes()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(100L))
      {
         f.filter.initialize();
         int dim = 2 * f.n + 3 * f.m;
         assertEquals(dim, f.filter.getStateDimension(), f.describe());
         DMatrixRMaj x = f.filter.getStateVector();
         DMatrixRMaj P = f.filter.getCovariance();
         assertEquals(dim, x.numRows, f.describe() + " x rows");
         assertEquals(1, x.numCols, f.describe() + " x cols");
         assertEquals(dim, P.numRows, f.describe() + " P rows");
         assertEquals(dim, P.numCols, f.describe() + " P cols");
      }
   }

   @Test
   public void testInferredDims()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(200L))
      {
         assertTrue(f.n > 0, f.describe() + " expected at least one filtered joint");
         assertTrue(f.m >= 2, f.describe() + " a pair needs at least two IMUs");
         assertEquals(2 * f.n + 3 * f.m, f.dim, f.describe());
      }
   }

   @Test
   public void testInitStateDefaultsZero()
   {
      // Encoders all zero ⇒ the seeded mean is exactly zero (q from encoders, q_dot and bias default to 0).
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(1L, 8, 1, 7);
      for (OneDoFJointBasics joint : f.filteredJoints)
         f.setEncoder(joint, 0.0);
      f.filter.initialize();
      DMatrixRMaj x = f.filter.getStateVector();
      for (int i = 0; i < x.numRows; i++)
         assertEquals(0.0, x.get(i, 0), 0.0, "x[" + i + "]");
   }

   @Test
   public void testQ0Seed()
   {
      // The position segment must be seeded from the encoder readings, in state order.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(2L, 8, 1, 7);
      double[] encoders = new double[f.n];
      for (int i = 0; i < f.n; i++)
      {
         encoders[i] = 0.11 * (i + 1);
         f.setEncoder(f.filteredJoints.get(i), encoders[i]);
      }
      f.filter.initialize();
      DMatrixRMaj x = f.filter.getStateVector();
      for (int i = 0; i < f.n; i++)
         assertEquals(encoders[i], x.get(i, 0), 1.0e-12, "q_hat[" + i + "]");
   }

   @Test
   public void testXOrdering()
   {
      // x must be exactly [q ; q_dot ; b_omega]: seed q from encoders, everything else zero at init.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(3L, 8, 1, 7);
      for (int i = 0; i < f.n; i++)
         f.setEncoder(f.filteredJoints.get(i), 1.0);
      f.filter.initialize();
      DMatrixRMaj x = f.filter.getStateVector();
      for (int i = 0; i < f.n; i++)
         assertEquals(1.0, x.get(i, 0), 1.0e-12, "q segment");
      for (int i = f.n; i < 2 * f.n; i++)
         assertEquals(0.0, x.get(i, 0), 0.0, "q_dot segment");
      for (int i = 2 * f.n; i < f.dim; i++)
         assertEquals(0.0, x.get(i, 0), 0.0, "bias segment");
   }

   @Test
   public void testMarginalBlocksMatchP()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(300L))
      {
         f.filter.initialize();
         DMatrixRMaj P = f.filter.getCovariance();
         int n = f.n;
         for (int i = 0; i < n; i++)
            assertEquals(INIT_POS_VAR, P.get(i, i), 1.0e-12, f.describe() + " position prior");
         for (int i = n; i < 2 * n; i++)
            assertEquals(INIT_VEL_VAR, P.get(i, i), 1.0e-12, f.describe() + " velocity prior");
         for (int i = 2 * n; i < f.dim; i++)
            assertEquals(INIT_BIAS_VAR, P.get(i, i), 1.0e-12, f.describe() + " bias prior");
      }
   }

   @Test
   public void testVelocityBlockExcludesBiasBlock()
   {
      // Regression guard mirroring the reference: the velocity marginal must be pure velocity, not the bias
      // block. Distinguish them by their different prior variances.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4L, 6, 1, 5);
      f.filter.initialize();
      DMatrixRMaj P = f.filter.getCovariance();
      for (int i = f.n; i < 2 * f.n; i++)
      {
         assertEquals(INIT_VEL_VAR, P.get(i, i), 1.0e-12);
         assertTrue(Math.abs(P.get(i, i) - INIT_BIAS_VAR) > 1.0e-6, "velocity prior must differ from bias prior");
      }
   }

   @Test
   public void testPSymmetricAndPSD()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(500L))
      {
         f.filter.initialize();
         DMatrixRMaj P = f.filter.getCovariance();
         assertSymmetric(P, 1.0e-12, f.describe() + " P symmetric");
         assertPositiveSemiDefinite(P, f.describe() + " P PSD");
      }
   }

   @Test
   public void testPriorConfidenceOrdering()
   {
      // Reference asserts the bias prior is tighter than both joint priors. This filter trusts the encoders
      // even more than the bias, so the actual ordering is position < bias < velocity — the relative
      // confidence intent is preserved (velocities are the least-known state at init).
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(6L, 8, 1, 7);
      f.filter.initialize();
      DMatrixRMaj P = f.filter.getCovariance();
      double pos = P.get(0, 0);
      double vel = P.get(f.n, f.n);
      double bias = P.get(2 * f.n, 2 * f.n);
      assertTrue(pos < bias, "position prior should be tighter than bias prior");
      assertTrue(bias < vel, "bias prior should be tighter than velocity prior");
   }

   @Test
   public void testPriorVariancesPositive()
   {
      // Analogue of test_default_params_values: every prior variance must be strictly positive.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(7L, 8, 1, 7);
      f.filter.initialize();
      DMatrixRMaj P = f.filter.getCovariance();
      for (int i = 0; i < f.dim; i++)
         assertTrue(P.get(i, i) > 0.0, "prior variance " + i + " must be > 0");
   }
}
