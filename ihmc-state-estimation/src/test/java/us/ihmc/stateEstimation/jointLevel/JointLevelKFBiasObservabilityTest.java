package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

/**
 * OBSERVABILITY of the base-IMU gyro bias — the root cause of the pelvis pitch drift (FINDINGS.md Part F,
 * jointkf_bias_observability.md).
 *
 * <p>The stacked gyro measurement is {@code z = J_stack(q)·q̇ + L(q)·b + noise}. The IMU-pair rows see the bias
 * ONLY as a rotated difference — {@code L_e·b = +R(c→J_e)·b_c − R(p→J_e)·b_p} — so L restricted to them is a
 * rotation-weighted signed incidence matrix of the IMU tree. Its nullspace is the 3-dimensional
 * <b>common-mode gauge</b>
 * <pre>
 *   N = { δb : δb_i = R(i←W)·β , β ∈ R³ }
 * </pre>
 * A single common rotational-rate offset, expressed consistently in each IMU's own frame, is invisible to a
 * model built purely from differences. The stance-anchor row's {@code +I₃} on the base bias is the ONLY absolute
 * bias observation in the whole filter, and hence the only thing that fixes that gauge.
 *
 * <p>These tests pin both halves of that statement, and the regression that made it bite: an anchor whose chain
 * contains unfiltered joints (Alex's ankles — no foot IMU ⇒ no shin→foot pair ⇒ ankles never became states) must
 * still work, because the unfiltered joints only have to be KNOWN, not ESTIMATED.
 */
public class JointLevelKFBiasObservabilityTest
{
   private static final long SEED = 20260712L;

   /**
    * With NO active anchor, the common-mode bias direction is in the nullspace of H: perturbing every IMU's bias
    * by the same world-frame vector β (expressed in each IMU's own frame) changes no measurement at all.
    *
    * <p>This is the defect. The filter cannot see this direction, so P along it grows without bound under Q and
    * the mean wanders — on hardware, all the way to ‖β‖ ≈ 0.17 rad/s, which the InEKF then subtracted from a
    * gyro reading ~0 and integrated straight into base pitch.
    */
   @Test
   public void testCommonModeBiasIsUnobservableWithoutAnchors()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(SEED))
      {
         f.filter.setTrustedFeetForTest(new ArrayList<>()); // no foot trusted => K = 0 => pairs only
         f.filter.buildStackedMeasurementForTest();
         DMatrixRMaj H = f.filter.getStackedMeasurementJacobian();

         DMatrixRMaj gauge = gaugeDirection(f, new Vector3D(0.013, -0.007, 0.021));
         DMatrixRMaj hDelta = new DMatrixRMaj(H.getNumRows(), 1);
         CommonOps_DDRM.mult(H, gauge, hDelta);

         assertEquals(0.0,
                      NormOps_DDRM.normF(hDelta),
                      1.0e-9,
                      "the common-mode bias is a NULLSPACE direction of H with no anchor active " + f.describe()
                      + " -- the pair rows see only bias DIFFERENCES, so this direction is unobservable");
      }
   }

   /**
    * One active anchor kills the gauge: the same perturbation now produces a nonzero residual, because the anchor
    * row carries {@code +I₃} on the base bias. {@code ‖H·δx‖} should be ≈ ‖β‖ per anchor (the anchor's bias block
    * is the identity and δq̇ = 0).
    */
   @Test
   public void testStanceAnchorFixesTheGauge()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(SEED))
      {
         Vector3D beta = new Vector3D(0.013, -0.007, 0.021);
         DMatrixRMaj gauge = gaugeDirection(f, beta);

         f.filter.setTrustedFeetForTest(new ArrayList<>(f.feet)); // foot planted => anchor active
         f.filter.buildStackedMeasurementForTest();
         DMatrixRMaj H = f.filter.getStackedMeasurementJacobian();

         DMatrixRMaj hDelta = new DMatrixRMaj(H.getNumRows(), 1);
         CommonOps_DDRM.mult(H, gauge, hDelta);

         assertEquals(beta.norm(),
                      NormOps_DDRM.normF(hDelta),
                      1.0e-9,
                      "with an anchor active the common-mode bias is OBSERVABLE " + f.describe()
                      + " -- the anchor's +I3 on the base bias is the gauge-fixing row, and it reads back exactly beta");
      }
   }

   /**
    * THE REGRESSION. Alex's real topology: the foot lies beyond the last IMU, so the chain joints between them
    * (the ankles) are NOT filter states. The anchor must remain usable — the unfiltered joints' measured
    * velocities are folded into z', and the {@code +I₃} that fixes the gauge is untouched.
    *
    * <p>Before the fix, {@code fa.usable} was set false whenever ANY chain joint was unfiltered, so on Alex both
    * anchors were dead on EVERY tick since day one and the bias gauge was never fixed.
    */
   @Test
   public void testAnchorIsUsableWhenChainHasUnfilteredJoints()
   {
      // 10 joints; IMUs on links after joints 1 and 5 (so joints 2..5 are filtered); foot after joint 9 =>
      // joints 6..9 are on the base->foot chain but are NOT states. This is the Alex ankle case.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairFootBeyondIMUs(SEED, 10, 1, 5, 9);

      List<RigidBodyBasics> chainHasUnfiltered = new ArrayList<>(f.feet);
      f.filter.setTrustedFeetForTest(chainHasUnfiltered);
      f.filter.buildStackedMeasurementForTest();

      assertEquals(1, f.filter.getActiveAnchorCountForTest(),
                   "the anchor must stay ACTIVE even though the ankles on its chain are not filter states");

      // And it must actually fix the gauge.
      Vector3D beta = new Vector3D(0.011, 0.004, -0.017);
      DMatrixRMaj gauge = gaugeDirection(f, beta);
      DMatrixRMaj H = f.filter.getStackedMeasurementJacobian();
      DMatrixRMaj hDelta = new DMatrixRMaj(H.getNumRows(), 1);
      CommonOps_DDRM.mult(H, gauge, hDelta);

      assertEquals(beta.norm(), NormOps_DDRM.normF(hDelta), 1.0e-9,
                   "an anchor with unfiltered chain joints still fixes the common-mode bias gauge");

      // Sanity: the same fixture with no trusted foot is unobservable again (the gauge is anchor-fixed, not
      // topology-fixed).
      f.filter.setTrustedFeetForTest(new ArrayList<>());
      f.filter.buildStackedMeasurementForTest();
      DMatrixRMaj H0 = f.filter.getStackedMeasurementJacobian();
      DMatrixRMaj h0Delta = new DMatrixRMaj(H0.getNumRows(), 1);
      CommonOps_DDRM.mult(H0, gauge, h0Delta);
      assertEquals(0.0, NormOps_DDRM.normF(h0Delta), 1.0e-9, "no anchor => gauge reopens");
   }

   /**
    * The unfiltered joints' encoder-velocity noise must be propagated into the anchor covariance by the
    * input-noise congruence {@code R_anchor = Σ_ε + J_U·diag(σ_qd²)·J_Uᵀ}. Over-trusting a noisy measured input
    * would feed that noise straight into the base gyro-bias estimate.
    */
   @Test
   public void testAnchorCovarianceIncludesUnfilteredJointNoise()
   {
      JointLevelKFTestFixture withUnfiltered = JointLevelKFTestFixture.singlePairFootBeyondIMUs(SEED, 10, 1, 5, 9);
      withUnfiltered.filter.setTrustedFeetForTest(new ArrayList<>(withUnfiltered.feet));
      withUnfiltered.filter.buildStackedMeasurementForTest();
      DMatrixRMaj R = withUnfiltered.filter.getStackedMeasurementNoise();

      // The anchor block is the trailing 3x3 (pairs occupy rows [0, 3E)).
      int a = R.getNumRows() - 3;
      double anchorTrace = R.get(a, a) + R.get(a + 1, a + 1) + R.get(a + 2, a + 2);

      // Sigma_eps alone would give trace = 3 * ANCHOR_VAR = 1.2e-3 (plus the base gyro's own Sigma, which is
      // tiny). With four unfiltered joints at sigma_qd = 0.1 rad/s and O(1) Jacobian columns, the congruence
      // must dominate it by orders of magnitude.
      assertTrue(anchorTrace > 1.0e-2,
                 "the anchor covariance must include the J_U diag(sigma_qd^2) J_U^T congruence for the unfiltered "
                 + "ankle velocities; got trace = " + anchorTrace);

      // A fixture with NO unfiltered chain joints must NOT pay that penalty.
      JointLevelKFTestFixture allFiltered = JointLevelKFTestFixture.singlePair(SEED, 10, 1, 9);
      allFiltered.filter.setTrustedFeetForTest(new ArrayList<>(allFiltered.feet));
      allFiltered.filter.buildStackedMeasurementForTest();
      DMatrixRMaj Rclean = allFiltered.filter.getStackedMeasurementNoise();
      int b = Rclean.getNumRows() - 3;
      double cleanTrace = Rclean.get(b, b) + Rclean.get(b + 1, b + 1) + Rclean.get(b + 2, b + 2);

      assertTrue(cleanTrace < anchorTrace,
                 "an all-filtered chain carries no unfiltered-velocity noise, so its anchor is tighter: "
                 + cleanTrace + " vs " + anchorTrace);
   }

   /**
    * Builds the gauge-direction state perturbation δx = (0, 0, δb) with δb_i = R(i←W)·β for every IMU, i.e. one
    * common world-frame rate offset β written in each IMU's own measurement frame.
    */
   private static DMatrixRMaj gaugeDirection(JointLevelKFTestFixture f, Vector3D betaWorld)
   {
      DMatrixRMaj delta = new DMatrixRMaj(f.dim, 1); // q and qd blocks stay zero
      RotationMatrix worldToImu = new RotationMatrix();
      Vector3D biasInImuFrame = new Vector3D();

      for (IMUSensorReadOnly imu : f.imus)
      {
         ReferenceFrame imuFrame = imu.getMeasurementFrame();
         worldToImu.set(imuFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()).getRotation());
         worldToImu.inverseTransform(betaWorld, biasInImuFrame); // R(imu <- world) * beta

         int col = f.filter.getBiasBlockColumn(imu);
         delta.set(col, 0, biasInImuFrame.getX());
         delta.set(col + 1, 0, biasInImuFrame.getY());
         delta.set(col + 2, 0, biasInImuFrame.getZ());
      }
      return delta;
   }
}
