package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertAllClose;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.identity;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.spd;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.trace;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

/**
 * Ported from {@code tests/jointKF/test_update.py}. Locks in the Joseph-form measurement update
 * ν = z − Hx⁻, S = HP⁻Hᵀ + R, K = P⁻Hᵀ S⁻¹, x⁺ = x⁻ + Kν, P⁺ = (I−KH)P⁻(I−KH)ᵀ + KRKᵀ — driven through the
 * package-private {@link JointLevelKFPreFilter#josephUpdate} against an explicit-inverse reference KF.
 */
public class JointLevelKFUpdateTest
{
   /** A generic k-row measurement (H, z, R) unrelated to geometry, to exercise the Joseph math directly. */
   private static DMatrixRMaj genericH(int k, int dim, long seed)
   {
      DMatrixRMaj H = new DMatrixRMaj(k, dim);
      for (int r = 0; r < k; r++)
         for (int c = 0; c < dim; c++)
            H.set(r, c, Math.sin(0.37 * (r * dim + c + 1) + seed));
      return H;
   }

   private static DMatrixRMaj seededPrior(JointLevelKFTestFixture f, long seed)
   {
      int dim = f.dim;
      DMatrixRMaj x = new DMatrixRMaj(dim, 1);
      for (int i = 0; i < dim; i++)
         x.set(i, 0, 0.1 * (i + 1));
      DMatrixRMaj P = spd(dim, seed);
      f.filter.setStateForTest(x, P);
      return P;
   }

   /** Explicit-inverse reference: returns {xPosterior, Pposterior}. */
   private static DMatrixRMaj[] referenceUpdate(DMatrixRMaj x, DMatrixRMaj P, DMatrixRMaj H, DMatrixRMaj z, DMatrixRMaj R)
   {
      int dim = x.numRows;
      int k = H.numRows;
      DMatrixRMaj Ht = new DMatrixRMaj(dim, k);
      CommonOps_DDRM.transpose(H, Ht);
      DMatrixRMaj PHt = new DMatrixRMaj(dim, k);
      CommonOps_DDRM.mult(P, Ht, PHt);
      DMatrixRMaj S = new DMatrixRMaj(k, k);
      CommonOps_DDRM.mult(H, PHt, S);
      CommonOps_DDRM.addEquals(S, R);
      DMatrixRMaj Sinv = new DMatrixRMaj(k, k);
      CommonOps_DDRM.invert(S, Sinv);
      DMatrixRMaj K = new DMatrixRMaj(dim, k);
      CommonOps_DDRM.mult(PHt, Sinv, K);
      DMatrixRMaj Hx = new DMatrixRMaj(k, 1);
      CommonOps_DDRM.mult(H, x, Hx);
      DMatrixRMaj nu = new DMatrixRMaj(k, 1);
      CommonOps_DDRM.subtract(z, Hx, nu);
      DMatrixRMaj xNew = new DMatrixRMaj(dim, 1);
      CommonOps_DDRM.mult(K, nu, xNew);
      CommonOps_DDRM.addEquals(xNew, x);
      DMatrixRMaj KH = new DMatrixRMaj(dim, dim);
      CommonOps_DDRM.mult(K, H, KH);
      DMatrixRMaj ImKH = identity(dim);
      CommonOps_DDRM.subtractEquals(ImKH, KH);
      DMatrixRMaj tmp = new DMatrixRMaj(dim, dim);
      CommonOps_DDRM.mult(ImKH, P, tmp);
      DMatrixRMaj Pnew = new DMatrixRMaj(dim, dim);
      CommonOps_DDRM.multTransB(tmp, ImKH, Pnew);
      DMatrixRMaj KR = new DMatrixRMaj(dim, k);
      CommonOps_DDRM.mult(K, R, KR);
      DMatrixRMaj KRKt = new DMatrixRMaj(dim, dim);
      CommonOps_DDRM.multTransB(KR, K, KRKt);
      CommonOps_DDRM.addEquals(Pnew, KRKt);
      return new DMatrixRMaj[] {xNew, Pnew};
   }

   @Test
   public void testShapesAndReferenceKF()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(3000L))
      {
         DMatrixRMaj P = seededPrior(f, 1);
         DMatrixRMaj x = f.filter.getStateVector();
         DMatrixRMaj H = genericH(3, f.dim, 2);
         DMatrixRMaj z = new DMatrixRMaj(3, 1);
         for (int i = 0; i < 3; i++)
            z.set(i, 0, 0.05 * (i + 1));
         DMatrixRMaj R = spd(3, 7);

         DMatrixRMaj[] ref = referenceUpdate(x, P, H, z, R);
         f.filter.josephUpdate(H, z, R);

         assertEquals(f.dim, f.filter.getStateVector().numRows, f.describe());
         assertAllClose(f.filter.getStateVector(), ref[0], 1.0e-6, f.describe() + " x⁺ matches reference KF");
         assertAllClose(f.filter.getCovariance(), ref[1], 1.0e-6, f.describe() + " P⁺ matches reference KF");
      }
   }

   @Test
   public void testCovarianceSymmetricPSD()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(3100L))
      {
         seededPrior(f, 3);
         DMatrixRMaj H = genericH(3, f.dim, 4);
         DMatrixRMaj z = new DMatrixRMaj(3, 1);
         DMatrixRMaj R = spd(3, 9);
         f.filter.josephUpdate(H, z, R);
         DMatrixRMaj P = f.filter.getCovariance();
         assertSymmetric(P, 1.0e-6, f.describe() + " P⁺ symmetric");
         assertPositiveSemiDefinite(P, f.describe() + " P⁺ PSD");
      }
   }

   @Test
   public void testCovarianceShrinks()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(3200L))
      {
         DMatrixRMaj prior = seededPrior(f, 5);
         DMatrixRMaj H = genericH(3, f.dim, 6);
         DMatrixRMaj z = new DMatrixRMaj(3, 1);
         DMatrixRMaj R = spd(3, 11);
         f.filter.josephUpdate(H, z, R);
         DMatrixRMaj post = f.filter.getCovariance();
         // A measurement cannot increase uncertainty: P⁻ − P⁺ is PSD, and the trace does not grow.
         DMatrixRMaj diff = new DMatrixRMaj(f.dim, f.dim);
         CommonOps_DDRM.subtract(prior, post, diff);
         assertPositiveSemiDefinite(diff, f.describe() + " P⁻ − P⁺ PSD");
         assertTrue(trace(post) <= trace(prior) + 1.0e-6, f.describe() + " trace(P⁺) ≤ trace(P⁻)");
      }
   }

   @Test
   public void testEncoderPullTinyR()
   {
      // With an extremely confident 1-D encoder measurement of a single position state, the posterior mean is
      // pulled onto the reading (the Java analogue of the reference's encoder-only tiny-R pull).
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(3300L, 8, 1, 7);
      seededPrior(f, 13);
      double target = 0.3 + f.filter.getStateVector().get(0, 0);
      DMatrixRMaj H = new DMatrixRMaj(1, f.dim);
      H.set(0, 0, 1.0); // observe position state 0
      DMatrixRMaj z = new DMatrixRMaj(1, 1);
      z.set(0, 0, target);
      DMatrixRMaj R = new DMatrixRMaj(1, 1);
      R.set(0, 0, 1.0e-16);
      f.filter.josephUpdate(H, z, R);
      assertEquals(target, f.filter.getStateVector().get(0, 0), 1.0e-3, "posterior position pulled onto the reading");
   }

   @Test
   public void testBiasObservability()
   {
      // A nonzero IMU (stacked-gyro) innovation moves the per-IMU residual-bias estimate. Single pair, no
      // trusted feet ⇒ the stacked measurement is exactly this pair's 3-row block.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(3400L, 8, 1, 7);
      f.imus.get(0).setAngularVelocity(0.05, -0.03, 0.02);
      f.imus.get(1).setAngularVelocity(-0.04, 0.06, -0.01);
      f.filter.initialize();
      f.filter.buildStackedMeasurementForTest();
      DMatrixRMaj H = f.filter.getStackedMeasurementJacobian();
      DMatrixRMaj z = f.filter.getStackedMeasurementResidual();
      DMatrixRMaj R = f.filter.getStackedMeasurementNoise();

      int parentBias = f.filter.getPairParentBiasColumn(0);
      int childBias = f.filter.getPairChildBiasColumn(0);
      DMatrixRMaj before = f.filter.getStateVector();
      f.filter.josephUpdate(H, z, R);
      DMatrixRMaj after = f.filter.getStateVector();

      double moved = 0.0;
      for (int i = 0; i < 3; i++)
         moved += Math.abs(after.get(parentBias + i, 0) - before.get(parentBias + i, 0))
               + Math.abs(after.get(childBias + i, 0) - before.get(childBias + i, 0));
      assertTrue(moved > 1.0e-9, "a nonzero IMU innovation must move the bias estimate");
   }

   @Test
   public void testDeterministic()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(3500L, 8, 1, 7);
      int dim = f.dim;
      DMatrixRMaj x0 = new DMatrixRMaj(dim, 1);
      for (int i = 0; i < dim; i++)
         x0.set(i, 0, 0.02 * (i + 1));
      DMatrixRMaj p0 = spd(dim, 21);
      DMatrixRMaj H = genericH(3, dim, 22);
      DMatrixRMaj z = new DMatrixRMaj(3, 1);
      DMatrixRMaj R = spd(3, 23);

      f.filter.setStateForTest(x0, p0);
      f.filter.josephUpdate(H, z, R);
      DMatrixRMaj xa = f.filter.getStateVector();
      DMatrixRMaj pa = f.filter.getCovariance();

      f.filter.setStateForTest(x0, p0);
      f.filter.josephUpdate(H, z, R);
      assertAllClose(f.filter.getStateVector(), xa, 0.0, "update deterministic (x)");
      assertAllClose(f.filter.getCovariance(), pa, 0.0, "update deterministic (P)");
   }
}
