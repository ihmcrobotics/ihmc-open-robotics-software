package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertAllClose;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.trace;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

/**
 * Ported from {@code tests/jointKF/test_filter.py}. Drives the full predict→update orchestration
 * ({@link JointLevelKFPreFilter#computeJointState()} phase 1 + {@link JointLevelKFPreFilter#computeImuBiases}
 * phase 2) over trajectories and checks the whole-filter invariants: covariance stays symmetric PSD and
 * bounded, the run is deterministic, encoder tracking holds, and the stance anchor makes the base bias
 * converge to a constant gyro offset.
 *
 * <p>The reference's {@code jax.lax.scan} orchestration and float64 checks do not apply; the per-piece
 * step-composition is covered by {@link JointLevelKFPredictTest} and {@link JointLevelKFUpdateTest}.</p>
 */
public class JointLevelKFFilterTest
{
   private static void tick(JointLevelKFTestFixture f)
   {
      f.filter.computeJointState();
      f.filter.computeImuBiases(f.feet);
   }

   @Test
   public void testTrajectoryFiniteAndShapes()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(5000L))
      {
         f.filter.initialize();
         for (int t = 0; t < 50; t++)
            tick(f);
         DMatrixRMaj x = f.filter.getStateVector();
         DMatrixRMaj P = f.filter.getCovariance();
         assertEquals(f.dim, x.numRows, f.describe());
         assertEquals(f.dim, P.numRows, f.describe());
         for (int i = 0; i < f.dim; i++)
         {
            assertTrue(Double.isFinite(x.get(i, 0)), f.describe() + " x finite");
            for (int j = 0; j < f.dim; j++)
               assertTrue(Double.isFinite(P.get(i, j)), f.describe() + " P finite");
         }
      }
   }

   @Test
   public void testCovariancePSDAlongTrajectory()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(5100L))
      {
         f.filter.initialize();
         for (int t = 0; t < 20; t++)
         {
            tick(f);
            DMatrixRMaj P = f.filter.getCovariance();
            assertSymmetric(P, 1.0e-6, f.describe() + " P symmetric at t=" + t);
            assertPositiveSemiDefinite(P, f.describe() + " P PSD at t=" + t);
         }
      }
   }

   @Test
   public void testDeterministic()
   {
      // Two filters with the same seed (⇒ identical geometry) and the same sensor stream run identically.
      JointLevelKFTestFixture a = JointLevelKFTestFixture.singlePair(5200L, 8, 1, 7);
      JointLevelKFTestFixture b = JointLevelKFTestFixture.singlePair(5200L, 8, 1, 7);
      for (JointLevelKFTestFixture f : new JointLevelKFTestFixture[] {a, b})
      {
         f.imus.get(0).setAngularVelocity(0.02, -0.01, 0.03);
         f.imus.get(1).setAngularVelocity(0.01, 0.02, -0.02);
         f.filter.initialize();
      }
      for (int t = 0; t < 30; t++)
      {
         tick(a);
         tick(b);
      }
      assertAllClose(a.filter.getStateVector(), b.filter.getStateVector(), 0.0, "deterministic trajectory (x)");
      assertAllClose(a.filter.getCovariance(), b.filter.getCovariance(), 0.0, "deterministic trajectory (P)");
   }

   @Test
   public void testEncoderTracking()
   {
      // Tight encoder noise ⇒ the position estimate tracks the encoder readings.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(5300L, 8, 1, 7);
      double[] target = new double[f.n];
      for (int i = 0; i < f.n; i++)
      {
         target[i] = 0.2 * (i + 1) - 0.5;
         f.setEncoder(f.filteredJoints.get(i), target[i]);
      }
      f.filter.initialize();
      for (int t = 0; t < 100; t++)
         tick(f);
      DMatrixRMaj x = f.filter.getStateVector();
      for (int i = 0; i < f.n; i++)
         assertEquals(target[i], x.get(i, 0), 5.0e-3, "estimated position tracks encoder " + i);
   }

   @Test
   public void testStancePhaseBiasConvergence()
   {
      // Static joints + a constant base-IMU gyro offset + a trusted stance foot ⇒ the residual-bias estimate
      // for the base IMU converges onto that offset (the stance anchor attributes the reading to bias).
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(5400L, 8, 1, 7);
      double ox = 0.01;
      double oy = -0.02;
      double oz = 0.03;
      for (JointLevelKFTestFixture.TestIMU imu : f.imus)
         imu.setAngularVelocity(ox, oy, oz);
      for (OneDoFJointBasics joint : f.filteredJoints)
         f.setEncoder(joint, 0.0);
      f.filter.initialize();
      for (int t = 0; t < 20_000; t++)
         tick(f);

      FrameVector3DReadOnly bias = f.filter.getAngularVelocityBiasInIMUFrame(f.imus.get(0));
      assertEquals(ox, bias.getX(), 2.0e-3, "base bias x → offset");
      assertEquals(oy, bias.getY(), 2.0e-3, "base bias y → offset");
      assertEquals(oz, bias.getZ(), 2.0e-3, "base bias z → offset");
   }

   @Test
   public void testCovarianceBounded()
   {
      // The covariance must not blow up over a long measurement-fed run.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(5500L, 8, 1, 7);
      f.filter.initialize();
      double initialTrace = trace(f.filter.getCovariance());
      for (int t = 0; t < 2000; t++)
         tick(f);
      double finalTrace = trace(f.filter.getCovariance());
      assertTrue(Double.isFinite(finalTrace) && finalTrace <= initialTrace * 10.0 + 1.0,
                 "trace(P) stayed bounded: " + finalTrace + " vs initial " + initialTrace);
   }
}
