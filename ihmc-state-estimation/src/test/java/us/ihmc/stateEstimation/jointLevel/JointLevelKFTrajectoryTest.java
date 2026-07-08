package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

/**
 * Behavioral tests for {@link JointLevelKFPreFilter} on a simple, analytically-consistent simulated motion:
 * a smooth per-joint sinusoid whose encoder positions and IMU gyros are both derived from the same true
 * {@code q}/{@code q̇} on the live mecano chain (see {@link JointLevelKFTestFixture#applyConsistentMotion}).
 *
 * <p>Only {@link JointLevelKFPreFilter#computeJointState()} (phase 1: predict + encoder + pair-gyro updates)
 * is driven here — <b>not</b> {@code computeImuBiases} — because the phase-2 stance anchor assumes a
 * non-rotating stance foot, which this free joint motion deliberately violates. Phase 1 is fully consistent
 * with the fed sensors, so it is the right surface for a tracking/convergence test.</p>
 *
 * <p>Also serves as regression coverage for the NaN hardening (transient bad input, poisoned IMU covariance,
 * singular innovation) and asserts the covariance develops the expected off-diagonal coupling structure.</p>
 */
public class JointLevelKFTrajectoryTest
{
   private static final double AMP = 0.10;                        // rad
   private static final double FREQ_HZ = 0.5;                     // Hz — slow enough for the constant-velocity model
   private static final double OMEGA = 2.0 * Math.PI * FREQ_HZ;   // rad/s
   private static final double DT = JointLevelKFTestFixture.DT;   // 1e-3 s

   /** Fills the true joint positions/velocities of the prescribed sinusoid at the given tick. */
   private static void trajectory(int tick, int n, double[] qToPack, double[] qdToPack)
   {
      double t = tick * DT;
      for (int i = 0; i < n; i++)
      {
         double phase = OMEGA * t + i * Math.PI / n;
         qToPack[i] = AMP * Math.sin(phase);
         qdToPack[i] = AMP * OMEGA * Math.cos(phase);
      }
   }

   @Test
   public void testPositionTracksTrajectory()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(9100L, 8, 1, 7);
      int n = f.n;
      double[] q = new double[n];
      double[] qd = new double[n];
      int ticks = 300;
      for (int k = 0; k < ticks; k++)
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
      }
      DMatrixRMaj x = f.filter.getStateVector();
      trajectory(ticks - 1, n, q, qd);
      for (int i = 0; i < n; i++)
         assertEquals(q[i], x.get(i, 0), 5.0e-3, "estimated position tracks the trajectory, joint " + i + " " + f.describe());
   }

   @Test
   public void testVelocityConverges()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(9101L, 8, 1, 7);
      int n = f.n;
      double[] q = new double[n];
      double[] qd = new double[n];
      int warmup = 500;
      int total = 3000;
      double[] maxAbsEstimatedVelocity = new double[n];
      for (int k = 0; k < total; k++)
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
         if (k >= warmup)
         {
            DMatrixRMaj x = f.filter.getStateVector();
            for (int i = 0; i < n; i++)
               maxAbsEstimatedVelocity[i] = Math.max(maxAbsEstimatedVelocity[i], Math.abs(x.get(n + i, 0)));
         }
      }
      DMatrixRMaj x = f.filter.getStateVector();
      trajectory(total - 1, n, q, qd);
      double peakTrueVelocity = AMP * OMEGA;
      for (int i = 0; i < n; i++)
      {
         assertEquals(qd[i], x.get(n + i, 0), 3.0e-2, "estimated velocity converges, joint " + i + " " + f.describe());
         // Guards against a silent "velocity never left its prior" regression: q̇ must genuinely be observed.
         assertTrue(maxAbsEstimatedVelocity[i] > 0.5 * peakTrueVelocity,
                    "velocity is actually observed (peak " + maxAbsEstimatedVelocity[i] + " vs true " + peakTrueVelocity + "), joint " + i);
      }
   }

   @Test
   public void testBiasStaysSmallWithZeroTrueBias()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(9102L, 8, 1, 7);
      int n = f.n;
      double[] q = new double[n];
      double[] qd = new double[n];
      for (int k = 0; k < 1500; k++)
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
      }
      for (JointLevelKFTestFixture.TestIMU imu : f.imus)
      {
         FrameVector3DReadOnly bias = f.filter.getAngularVelocityBiasInIMUFrame(imu);
         assertTrue(bias.norm() < 5.0e-3, "estimated bias stays small (true bias is zero): " + imu.getSensorName() + " norm=" + bias.norm());
      }
   }

   @Test
   public void testCovarianceStaysPsdAndBounded()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(9103L, 8, 1, 7);
      int n = f.n;
      double[] q = new double[n];
      double[] qd = new double[n];

      int warmup = 200;
      for (int k = 0; k < warmup; k++)
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
      }
      double traceWarmup = JointLevelKFTestFixture.trace(f.filter.getCovariance());

      for (int k = warmup; k < 2000; k++)
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
         if (k % 50 == 0)
         {
            DMatrixRMaj p = f.filter.getCovariance();
            JointLevelKFTestFixture.assertSymmetric(p, 1.0e-6, "covariance symmetric along trajectory");
            JointLevelKFTestFixture.assertPositiveSemiDefinite(p, "covariance PSD along trajectory");
         }
      }
      double traceFinal = JointLevelKFTestFixture.trace(f.filter.getCovariance());
      assertTrue(traceFinal <= 10.0 * traceWarmup + 1.0, "covariance trace stays bounded (final " + traceFinal + " vs warmup " + traceWarmup + ")");
   }

   @Test
   public void testTransientNonFiniteInputRecovers()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(9104L, 8, 1, 7);
      int n = f.n;
      double[] q = new double[n];
      double[] qd = new double[n];

      for (int k = 0; k < 100; k++) // clean warm-up
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
      }

      for (int k = 100; k < 105; k++) // bad window: NaN gyro on one IMU + NaN encoder on one joint
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.imus.get(0).setAngularVelocity(Double.NaN, Double.NaN, Double.NaN);
         f.sensorMap.setPosition(f.filteredJoints.get(0), Double.NaN);
         f.filter.computeJointState();
         assertAllFinite(f.filter.getStateVector(), "state finite through the bad-input window (updates skipped)");
         assertAllFinite(f.filter.getCovariance(), "covariance finite through the bad-input window");
      }

      for (int k = 105; k < 1105; k++) // restore consistent input and let it re-converge
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
      }
      DMatrixRMaj x = f.filter.getStateVector();
      trajectory(1104, n, q, qd);
      for (int i = 0; i < n; i++)
      {
         assertEquals(q[i], x.get(i, 0), 5.0e-3, "position re-converges after transient NaN, joint " + i);
         assertEquals(qd[i], x.get(n + i, 0), 3.0e-2, "velocity re-converges after transient NaN, joint " + i);
      }
   }

   @Test
   public void testPoisonedBiasCovarianceDoesNotLatchNaN()
   {
      // IMU 0's bias process-noise covariance is non-finite BEFORE construction: the construction guard must
      // skip it so Q/F stay finite, and the per-tick R3 guard must keep its pair update out of x/P.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairPoisonBias(9105L, 8, 1, 7, 0);
      assertAllFinite(f.filter.getProcessNoise(), "process noise Q finite despite a poisoned IMU covariance");
      assertAllFinite(f.filter.getTransitionMatrix(), "transition F finite");

      int n = f.n;
      double[] q = new double[n];
      double[] qd = new double[n];
      for (int k = 0; k < 200; k++)
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
      }
      assertAllFinite(f.filter.getStateVector(), "state stays finite with a poisoned IMU covariance");
      assertAllFinite(f.filter.getCovariance(), "covariance stays finite with a poisoned IMU covariance");
   }

   @Test
   public void testSingularInnovationIsSkippedNotLatched()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(9106L, 8, 1, 7);
      int n = f.n;
      int dim = f.dim;
      double[] q0 = new double[n];
      double[] qd0 = new double[n];
      trajectory(0, n, q0, qd0); // use only the positions; drive from rest
      double[] zeros = new double[n];
      f.applyConsistentMotion(q0, zeros);
      f.filter.initialize();
      f.filter.predict(); // finite, positive-definite prior

      DMatrixRMaj xBefore = f.filter.getStateVector();
      DMatrixRMaj pBefore = f.filter.getCovariance();

      // Rank-deficient H with zero measurement noise → S = H P Hᵀ is singular (duplicate row + zero row).
      DMatrixRMaj hm = new DMatrixRMaj(3, dim);
      int velocityIndex = n; // velocity of joint 0
      hm.set(0, velocityIndex, 1.0);
      hm.set(1, velocityIndex, 1.0); // duplicate of row 0
      DMatrixRMaj zm = new DMatrixRMaj(3, 1);
      DMatrixRMaj rm = new DMatrixRMaj(3, 3); // exactly zero → no regularization, S stays singular
      f.filter.josephUpdate(hm, zm, rm);

      DMatrixRMaj xAfter = f.filter.getStateVector();
      DMatrixRMaj pAfter = f.filter.getCovariance();
      assertAllFinite(xAfter, "state finite after a singular innovation update");
      assertAllFinite(pAfter, "covariance finite after a singular innovation update");
      JointLevelKFTestFixture.assertAllClose(xAfter, xBefore, 0.0, "singular update is skipped, state unchanged");
      JointLevelKFTestFixture.assertAllClose(pAfter, pBefore, 0.0, "singular update is skipped, covariance unchanged");
   }

   @Test
   public void testCovarianceEmbedsKinematicCoupling()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(9107L, 8, 1, 7);
      int n = f.n;
      double[] q0 = new double[n];
      double[] qd0 = new double[n];
      trajectory(0, n, q0, qd0);
      double[] zeros = new double[n];
      f.applyConsistentMotion(q0, zeros);
      f.filter.initialize();
      f.filter.predict();

      // After one predict: within-joint q↔q̇ coupling exists (CWNA accel noise + F double-integrator);
      // cross-joint velocity coupling does NOT yet (F and Q are per-joint block-diagonal across joints).
      DMatrixRMaj p0 = f.filter.getCovariance();
      for (int i = 0; i < n; i++)
         assertTrue(Math.abs(p0.get(i, n + i)) > 1.0e-4, "within-joint q-qdot coupling present after predict, joint " + i);
      for (int i = 0; i < n; i++)
         for (int j = i + 1; j < n; j++)
            assertTrue(Math.abs(p0.get(n + i, n + j)) < 1.0e-10,
                       "no cross-joint velocity coupling before any pair update (" + i + "," + j + ")");

      // After pair-gyro updates over a moving trajectory: the shared Jacobian J(q) embeds the kinematic tree
      // as cross-joint velocity covariance.
      double[] q = new double[n];
      double[] qd = new double[n];
      for (int k = 0; k < 200; k++)
      {
         trajectory(k, n, q, qd);
         f.applyConsistentMotion(q, qd);
         f.filter.computeJointState();
      }
      DMatrixRMaj p1 = f.filter.getCovariance();
      JointLevelKFTestFixture.assertSymmetric(p1, 1.0e-6, "coupled covariance is still symmetric");
      JointLevelKFTestFixture.assertPositiveSemiDefinite(p1, "coupled covariance is still PSD");

      double maxCrossCorrelation = 0.0;
      for (int i = 0; i < n; i++)
         for (int j = i + 1; j < n; j++)
         {
            double denom = Math.sqrt(p1.get(n + i, n + i) * p1.get(n + j, n + j));
            if (denom > 0.0)
               maxCrossCorrelation = Math.max(maxCrossCorrelation, Math.abs(p1.get(n + i, n + j)) / denom);
         }
      assertTrue(maxCrossCorrelation > 0.02,
                 "kinematic tree embeds as cross-joint velocity coupling (max velocity correlation " + maxCrossCorrelation + ")");
      for (int i = 0; i < n; i++)
         assertTrue(Math.abs(p1.get(i, n + i)) > 1.0e-6, "within-joint q-qdot coupling persists, joint " + i);
   }

   private static void assertAllFinite(DMatrixRMaj matrix, String message)
   {
      for (int i = 0; i < matrix.getNumElements(); i++)
         assertTrue(Double.isFinite(matrix.get(i)), message + " — non-finite at linear index " + i);
   }
}
