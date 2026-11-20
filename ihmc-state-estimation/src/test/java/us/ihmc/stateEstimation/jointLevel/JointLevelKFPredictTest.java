package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertAllClose;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.block;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.scaledIdentity;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.spd;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

/**
 * Ported from {@code tests/jointKF/test_predict.py}. Locks in the EKF time update x⁻ = F x, P⁻ = F P Fᵀ + Q,
 * driven through the package-private {@link JointLevelKFPreFilter#predict()} seam from a seeded prior.
 */
public class JointLevelKFPredictTest
{
   private static final double DT = JointLevelKFTestFixture.DT;
   private static final double IMU_BIAS_VAR = JointLevelKFTestFixture.IMU_BIAS_PROCESS_VAR;

   /** Seeds a prior with distinct, recognizable mean segments and an SPD covariance, then returns the filter. */
   private static JointLevelKFTestFixture seededPrior(JointLevelKFTestFixture f, long seed)
   {
      int n = f.n;
      int dim = f.dim;
      DMatrixRMaj x = new DMatrixRMaj(dim, 1);
      for (int i = 0; i < n; i++)
         x.set(i, 0, i + 1.0);
      for (int i = 0; i < n; i++)
         x.set(n + i, 0, i + 1.0 + 100.0);
      for (int i = 0; i < 3 * f.m; i++)
         x.set(2 * n + i, 0, i + 1.0 + 1000.0);
      f.filter.setStateForTest(x, spd(dim, seed));
      return f;
   }

   @Test
   public void testShapesPreserved()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(2000L))
      {
         seededPrior(f, 0);
         f.filter.predict();
         assertEquals(f.dim, f.filter.getStateVector().numRows, f.describe());
         DMatrixRMaj P = f.filter.getCovariance();
         assertEquals(f.dim, P.numRows, f.describe());
         assertEquals(f.dim, P.numCols, f.describe());
      }
   }

   @Test
   public void testMeanPropagationExact()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(2100L))
      {
         seededPrior(f, 1);
         DMatrixRMaj xBefore = f.filter.getStateVector();
         f.filter.predict();
         DMatrixRMaj xAfter = f.filter.getStateVector();
         int n = f.n;
         for (int i = 0; i < n; i++)
            assertEquals(xBefore.get(i, 0) + DT * xBefore.get(n + i, 0), xAfter.get(i, 0), 1.0e-9, f.describe() + " q += dt·q̇");
         for (int i = n; i < 2 * n; i++)
            assertEquals(xBefore.get(i, 0), xAfter.get(i, 0), 1.0e-12, f.describe() + " q̇ constant");
         for (int i = 2 * n; i < f.dim; i++)
            assertEquals(xBefore.get(i, 0), xAfter.get(i, 0), 1.0e-12, f.describe() + " bias constant");
      }
   }

   @Test
   public void testCovarianceMatchesBuilders()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(2200L))
      {
         seededPrior(f, 2);
         DMatrixRMaj P = f.filter.getCovariance();
         DMatrixRMaj F = f.filter.getTransitionMatrix();
         DMatrixRMaj Q = f.filter.getProcessNoise();
         f.filter.predict();
         DMatrixRMaj actual = f.filter.getCovariance();

         // expected = F P Fᵀ + Q
         DMatrixRMaj FP = new DMatrixRMaj(f.dim, f.dim);
         CommonOps_DDRM.mult(F, P, FP);
         DMatrixRMaj expected = new DMatrixRMaj(f.dim, f.dim);
         CommonOps_DDRM.multTransB(FP, F, expected);
         CommonOps_DDRM.addEquals(expected, Q);
         assertAllClose(actual, expected, 1.0e-6, f.describe() + " P⁻ = F P Fᵀ + Q");
      }
   }

   @Test
   public void testCovarianceSymmetricPSD()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(2300L))
      {
         seededPrior(f, 3);
         f.filter.predict();
         DMatrixRMaj P = f.filter.getCovariance();
         assertSymmetric(P, 1.0e-6, f.describe() + " P symmetric");
         assertPositiveSemiDefinite(P, f.describe() + " P PSD");
      }
   }

   @Test
   public void testBiasBlockGrowthDiagonal()
   {
      // F is identity on the bias block and there is no F-coupling between bias and joints, so the bias
      // marginal grows by exactly the bias process noise: Σ_b⁺ − Σ_b⁻ = dt · Σ_imu.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(2400L, 8, 1, 7);
      seededPrior(f, 4);
      int n = f.n;
      int mDof = 3 * f.m;
      DMatrixRMaj before = block(f.filter.getCovariance(), 2 * n, 2 * n, mDof, mDof);
      f.filter.predict();
      DMatrixRMaj after = block(f.filter.getCovariance(), 2 * n, 2 * n, mDof, mDof);
      DMatrixRMaj increment = new DMatrixRMaj(mDof, mDof);
      CommonOps_DDRM.subtract(after, before, increment);
      assertAllClose(increment, scaledIdentity(mDof, DT * IMU_BIAS_VAR), 1.0e-9, "Σ_b increment = dt·Σ_imu");
   }

   @Test
   public void testDeterministic()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(2500L, 8, 1, 7);
      int dim = f.dim;
      DMatrixRMaj x0 = new DMatrixRMaj(dim, 1);
      for (int i = 0; i < dim; i++)
         x0.set(i, 0, 0.01 * (i + 1));
      DMatrixRMaj p0 = spd(dim, 5);

      f.filter.setStateForTest(x0, p0);
      f.filter.predict();
      DMatrixRMaj xa = f.filter.getStateVector();
      DMatrixRMaj pa = f.filter.getCovariance();

      f.filter.setStateForTest(x0, p0);
      f.filter.predict();
      assertAllClose(f.filter.getStateVector(), xa, 0.0, "predict is deterministic (x)");
      assertAllClose(f.filter.getCovariance(), pa, 0.0, "predict is deterministic (P)");
   }

   @Test
   public void testDiffusePositionVarianceGrows()
   {
      // A few measurement-free predicts from init_state keep P PSD and grow the position marginal.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(2600L, 8, 1, 7);
      f.filter.initialize();
      int n = f.n;
      double[] prev = new double[n];
      for (int i = 0; i < n; i++)
         prev[i] = f.filter.getCovariance().get(i, i);

      for (int step = 0; step < 5; step++)
      {
         f.filter.predict();
         DMatrixRMaj P = f.filter.getCovariance();
         assertPositiveSemiDefinite(P, "P PSD during diffuse predicts");
         for (int i = 0; i < n; i++)
         {
            double cur = P.get(i, i);
            assertTrue(cur >= prev[i] - 1.0e-9, "position variance must not shrink under measurement-free predict");
            prev[i] = cur;
         }
      }
   }
}
