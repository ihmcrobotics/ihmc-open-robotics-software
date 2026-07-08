package us.ihmc.stateEstimation.jointLevel;

import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertAllClose;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.spd;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

/**
 * THE decisive oracle for the Rev. 2 stacked gyro measurement (SPEC §9): the stacked update must equal a
 * reference KF that measures the RAW per-IMU gyros — with mutually INDEPENDENT (block-diagonal) noise — plus a
 * near-zero absolute-rate constraint per trusted foot, over a state augmented with a nuisance base angular
 * velocity ω_base, then marginalizes ω_base out.
 *
 * <p>The reference is built from a completely different decomposition than the filter: it uses absolute
 * base→IMU angular Jacobians and base→IMU mounting rotations and per-IMU independent noise, with the
 * shared-IMU / shared-base correlation of R_g emerging ONLY from marginalizing the common nuisance ω_base
 * (the standard "difference = marginalize the common mode" identity). So a sign, frame, block-placement, or
 * R_g-correlation error in the filter's L / H_g / R_g assembly (the "silent" convention traps of SPEC §9)
 * makes this posterior disagree. Agreement to round-off over many randomized ticks pins all of them at once.</p>
 */
public class JointLevelKFStackedOracleTest
{
   private static final double ANCHOR_VAR = 4.0e-4; // must match JointLevelKFPreFilter.ANCHOR_VAR (Sigma_eps)

   @Test
   public void testStackedUpdateMatchesNuisanceMarginalizedReference()
   {
      // Two pairs sharing the middle IMU (a chain a-b-c → base = a), one stance foot on the far link: exercises
      // shared-IMU pair cross-covariance AND a base-referencing anchor in one stacked block.
      Random random = new Random(90000L);
      for (int trial = 0; trial < 12; trial++)
      {
         JointLevelKFTestFixture f = JointLevelKFTestFixture.twoPairs(90100L + trial, 10, 1, 5, 9);

         // Arbitrary (not necessarily consistent) raw gyros — the equivalence is algebraic, not motion-specific.
         for (JointLevelKFTestFixture.TestIMU imu : f.imus)
            imu.setAngularVelocity(0.4 * (random.nextDouble() - 0.5),
                                   0.4 * (random.nextDouble() - 0.5),
                                   0.4 * (random.nextDouble() - 0.5));

         DMatrixRMaj muX = new DMatrixRMaj(f.dim, 1);
         for (int i = 0; i < f.dim; i++)
            muX.set(i, 0, 0.05 * (i + 1) - 0.1);
         DMatrixRMaj Pxx = spd(f.dim, 700L + trial);

         List<RigidBodyBasics> activeFeet = new ArrayList<>(f.feet); // both/all trusted this tick

         // ---- Reference: augmented raw-gyro KF, ω_base marginalized (information form, improper ω_base prior).
         DMatrixRMaj[] ref = referenceMarginalized(f, muX, Pxx, activeFeet);

         // ---- Filter: the actual stacked update from the same prior and the same raw gyros / trusted feet.
         f.filter.setStateForTest(muX, Pxx);
         f.filter.setTrustedFeetForTest(activeFeet);
         f.filter.buildStackedMeasurementForTest();
         f.filter.josephUpdate(f.filter.getStackedMeasurementJacobian(),
                               f.filter.getStackedMeasurementResidual(),
                               f.filter.getStackedMeasurementNoise());

         assertAllClose(f.filter.getStateVector(), ref[0], 1.0e-5,
                        "trial " + trial + " x⁺ matches nuisance-marginalized reference");
         assertAllClose(f.filter.getCovariance(), ref[1], 1.0e-5,
                        "trial " + trial + " P⁺ matches nuisance-marginalized reference");
      }
   }

   @Test
   public void testPairsOnlyMatchesReference()
   {
      // No trusted feet ⇒ K = 0 (pairs-only). The base bias / ω_base common mode is unobservable (gauge), but
      // the reference's improper ω_base prior handles that (the marginalized posterior over the observable
      // subspace still matches; the unobservable bias-gauge direction is governed identically by the prior).
      Random random = new Random(91000L);
      for (int trial = 0; trial < 8; trial++)
      {
         JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairMassMatrix(91100L + trial, 8, 1, 6);
         for (JointLevelKFTestFixture.TestIMU imu : f.imus)
            imu.setAngularVelocity(0.3 * (random.nextDouble() - 0.5),
                                   0.3 * (random.nextDouble() - 0.5),
                                   0.3 * (random.nextDouble() - 0.5));
         DMatrixRMaj muX = new DMatrixRMaj(f.dim, 1);
         for (int i = 0; i < f.dim; i++)
            muX.set(i, 0, 0.02 * (i + 1));
         DMatrixRMaj Pxx = spd(f.dim, 800L + trial);

         DMatrixRMaj[] ref = referenceMarginalized(f, muX, Pxx, new ArrayList<>());

         f.filter.setStateForTest(muX, Pxx);
         f.filter.setTrustedFeetForTest(new ArrayList<>());
         f.filter.buildStackedMeasurementForTest();
         f.filter.josephUpdate(f.filter.getStackedMeasurementJacobian(),
                               f.filter.getStackedMeasurementResidual(),
                               f.filter.getStackedMeasurementNoise());

         assertAllClose(f.filter.getStateVector(), ref[0], 1.0e-5, "trial " + trial + " pairs-only x⁺ matches reference");
         assertAllClose(f.filter.getCovariance(), ref[1], 1.0e-5, "trial " + trial + " pairs-only P⁺ matches reference");
      }
   }

   /**
    * Reference posterior over x = [q; q̇; b] from a KF that (i) augments the state with a nuisance base angular
    * velocity ω_base (in the base IMU frame), (ii) measures every IMU's RAW gyro
    * z̃_i = R(base→I_i) ω_base + J_abs,i(q) q̇ + b_i + n_i (independent noise Σ_i), (iii) measures each trusted
    * foot's absolute rate ω_base + J_leg,k q̇ ≈ 0 (independent noise Σ_ε), then marginalizes ω_base. Uses the
    * information form with an improper (zero-information) prior on ω_base so it is the exact γ→∞ limit the
    * filter (which has no ω_base state) represents.
    */
   private static DMatrixRMaj[] referenceMarginalized(JointLevelKFTestFixture f, DMatrixRMaj muX, DMatrixRMaj Pxx, List<RigidBodyBasics> activeFeet)
   {
      int dim = f.dim;
      int n = f.n;
      int D = dim + 3; // + ω_base
      int M = 3 * f.imus.size() + 3 * activeFeet.size();

      DMatrixRMaj H = new DMatrixRMaj(M, D);
      DMatrixRMaj z = new DMatrixRMaj(M, 1);
      DMatrixRMaj R = new DMatrixRMaj(M, M);
      IMUSensorReadOnly base = f.filter.getBaseIMU();
      GeometricJacobianCalculator jac = new GeometricJacobianCalculator();
      RigidBodyTransform transform = new RigidBodyTransform();
      DMatrixRMaj rot = new DMatrixRMaj(3, 3);
      DMatrixRMaj sigma = new DMatrixRMaj(3, 3);

      int row = 0;
      for (IMUSensorReadOnly imu : f.imus)
      {
         Vector3DReadOnly w = imu.getAngularVelocityMeasurement();
         z.set(row, 0, w.getX());
         z.set(row + 1, 0, w.getY());
         z.set(row + 2, 0, w.getZ());

         // ω_base columns: R(base measurement frame → IMU measurement frame).
         base.getMeasurementFrame().getTransformToDesiredFrame(transform, imu.getMeasurementFrame());
         JointLevelKFPreFilter.set_matrix(rot, transform.getRotation());
         for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
               H.set(row + a, dim + b, rot.get(a, b));

         // bias columns: +I3 at this IMU's bias block.
         int biasCol = f.filter.getBiasBlockColumn(imu);
         for (int a = 0; a < 3; a++)
            H.set(row + a, biasCol + a, 1.0);

         // q̇ columns: absolute angular Jacobian base→IMU link, expressed in the IMU frame (0 for the base IMU).
         if (imu != base)
         {
            jac.setKinematicChain(base.getMeasurementLink(), imu.getMeasurementLink());
            jac.setJacobianFrame(imu.getMeasurementFrame());
            jac.reset();
            DMatrixRMaj J = jac.getJacobianMatrix();
            List<OneDoFJointBasics> path = us.ihmc.mecano.tools.MultiBodySystemTools.filterJoints(jac.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);
            for (int c = 0; c < path.size(); c++)
            {
               int qdCol = n + f.filter.getJointStateIndex(path.get(c));
               for (int a = 0; a < 3; a++)
                  H.set(row + a, qdCol, J.get(a, c));
            }
         }

         imu.getAngularVelocityNoiseCovariance(sigma); // independent per-IMU white noise Σ_i
         for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
               R.set(row + a, row + b, sigma.get(a, b));
         row += 3;
      }

      for (RigidBodyBasics foot : activeFeet)
      {
         // Foot absolute rate ω_foot = ω_base + J_leg q̇ (base frame), asserted ≈ 0 with covariance Σ_ε.
         jac.setKinematicChain(base.getMeasurementLink(), foot);
         jac.setJacobianFrame(base.getMeasurementFrame());
         jac.reset();
         DMatrixRMaj J = jac.getJacobianMatrix();
         List<OneDoFJointBasics> path = us.ihmc.mecano.tools.MultiBodySystemTools.filterJoints(jac.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);
         for (int c = 0; c < path.size(); c++)
         {
            int qdCol = n + f.filter.getJointStateIndex(path.get(c));
            for (int a = 0; a < 3; a++)
               H.set(row + a, qdCol, J.get(a, c));
         }
         for (int a = 0; a < 3; a++)
            H.set(row + a, dim + a, 1.0); // +I3 on ω_base
         // z stays 0 (foot rate ≈ 0)
         for (int a = 0; a < 3; a++)
            R.set(row + a, row + a, ANCHOR_VAR);
         row += 3;
      }

      // Information-form update: Λ = Λ0 + Hᵀ R⁻¹ H, η = η0 + Hᵀ R⁻¹ z, with Λ0/η0 carrying the x prior only
      // (zero information on ω_base — the exact improper-prior limit the filter represents). Marginalizing
      // ω_base is then just reading the x-block of Λ⁻¹ and of the mean.
      DMatrixRMaj PxxInv = new DMatrixRMaj(dim, dim);
      CommonOps_DDRM.invert(Pxx, PxxInv);
      DMatrixRMaj Lambda = new DMatrixRMaj(D, D);
      for (int i = 0; i < dim; i++)
         for (int j = 0; j < dim; j++)
            Lambda.set(i, j, PxxInv.get(i, j));
      DMatrixRMaj eta = new DMatrixRMaj(D, 1);
      DMatrixRMaj etaX = new DMatrixRMaj(dim, 1);
      CommonOps_DDRM.mult(PxxInv, muX, etaX);
      for (int i = 0; i < dim; i++)
         eta.set(i, 0, etaX.get(i, 0));

      DMatrixRMaj Rinv = new DMatrixRMaj(M, M);
      CommonOps_DDRM.invert(R, Rinv);
      DMatrixRMaj HtRinv = new DMatrixRMaj(D, M);
      CommonOps_DDRM.multTransA(H, Rinv, HtRinv);
      DMatrixRMaj LambdaMeas = new DMatrixRMaj(D, D);
      CommonOps_DDRM.mult(HtRinv, H, LambdaMeas);
      CommonOps_DDRM.addEquals(Lambda, LambdaMeas);
      DMatrixRMaj etaMeas = new DMatrixRMaj(D, 1);
      CommonOps_DDRM.mult(HtRinv, z, etaMeas);
      CommonOps_DDRM.addEquals(eta, etaMeas);

      DMatrixRMaj Sigma = new DMatrixRMaj(D, D);
      CommonOps_DDRM.invert(Lambda, Sigma);
      DMatrixRMaj mu = new DMatrixRMaj(D, 1);
      CommonOps_DDRM.mult(Sigma, eta, mu);

      DMatrixRMaj muXPost = new DMatrixRMaj(dim, 1);
      DMatrixRMaj PxxPost = new DMatrixRMaj(dim, dim);
      for (int i = 0; i < dim; i++)
      {
         muXPost.set(i, 0, mu.get(i, 0));
         for (int j = 0; j < dim; j++)
            PxxPost.set(i, j, Sigma.get(i, j));
      }
      return new DMatrixRMaj[] {muXPost, PxxPost};
   }
}
