package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;

/**
 * Properties of the hand-over seed ({@link JointLevelKFPreFilter#seedFromHandover}), used when the invariant
 * estimator is switched in after standing by cold behind the DRC estimator.
 *
 * <p>The contract being pinned: the <em>mean</em> comes from the estimator that was already running (joint
 * q/q̇ off the model, per-IMU gyro bias from its bias provider), while the <em>covariance</em> does not — it
 * returns to exactly its startup value, because the two estimators share no state parameterization and so no
 * correlation of this filter's layout is recoverable from theirs.</p>
 *
 * @author Lucas Libshutz
 */
public class JointLevelKFHandoverSeedTest
{
   private static final double TOL = 1.0e-12;

   /** Every IMU gets the same bias, so the assertions do not depend on the filter's internal IMU ordering. */
   private static final class UniformBiasProvider implements IMUBiasProvider
   {
      private final FrameVector3D angularVelocityBias;

      private UniformBiasProvider(double x, double y, double z)
      {
         angularVelocityBias = new FrameVector3D(ReferenceFrame.getWorldFrame(), x, y, z);
      }

      @Override
      public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
      {
         return angularVelocityBias;
      }

      @Override
      public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
      {
         return angularVelocityBias;
      }

      @Override
      public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
      {
         return null;
      }

      @Override
      public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
      {
         return null;
      }
   }

   /**
    * The point of the whole exercise: after minutes of standby the covariance must not be whatever it happened
    * to converge to before, nor whatever it drifted to since — it must be the startup block-diagonal again.
    */
   @Test
   public void testHandoverSeedRestoresStartupCovariance()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(1234L, 4, 0, 3);
      f.filter.initialize();
      DMatrixRMaj startupCovariance = f.filter.getCovariance();

      // Run the filter so P picks up real correlation structure (and stops being block-diagonal).
      double[] qTrue = new double[f.n];
      double[] qdTrue = new double[f.n];
      for (int tick = 0; tick < 50; tick++)
      {
         for (int i = 0; i < f.n; i++)
         {
            qTrue[i] = 0.05 * Math.sin(0.01 * tick + i);
            qdTrue[i] = 0.05 * 0.01 * Math.cos(0.01 * tick + i);
         }
         f.applyConsistentMotion(qTrue, qdTrue);
         f.filter.computeJointState();
      }
      DMatrixRMaj evolvedCovariance = f.filter.getCovariance();
      assertFalse(isAllClose(evolvedCovariance, startupCovariance, 1.0e-9),
                  "Precondition: 50 ticks should have moved P away from its startup value. " + f.describe());

      f.filter.seedFromHandover(null);

      DMatrixRMaj seededCovariance = f.filter.getCovariance();
      JointLevelKFTestFixture.assertAllClose(seededCovariance, startupCovariance, TOL,
                                             "Hand-over seed must restore the startup covariance. " + f.describe());
      JointLevelKFTestFixture.assertSymmetric(seededCovariance, TOL, f.describe());
      JointLevelKFTestFixture.assertPositiveSemiDefinite(seededCovariance, f.describe());
   }

   /** All cross-covariances go, explicitly: the correlation the outgoing estimator built is not transferable. */
   @Test
   public void testHandoverSeedZeroesCrossCovariances()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4321L, 4, 0, 3);
      f.filter.initialize();

      double[] qTrue = new double[f.n];
      double[] qdTrue = new double[f.n];
      for (int tick = 0; tick < 50; tick++)
      {
         for (int i = 0; i < f.n; i++)
         {
            qTrue[i] = 0.05 * Math.sin(0.01 * tick + i);
            qdTrue[i] = 0.05 * 0.01 * Math.cos(0.01 * tick + i);
         }
         f.applyConsistentMotion(qTrue, qdTrue);
         f.filter.computeJointState();
      }

      f.filter.seedFromHandover(new UniformBiasProvider(0.001, -0.002, 0.003));

      DMatrixRMaj P = f.filter.getCovariance();
      for (int row = 0; row < f.dim; row++)
      {
         for (int col = 0; col < f.dim; col++)
         {
            if (row == col)
               continue;
            assertEquals(0.0, P.get(row, col), TOL, "P(" + row + "," + col + ") must be zero after a hand-over seed. " + f.describe());
         }
      }
      for (int i = 0; i < f.dim; i++)
         assertTrue(P.get(i, i) > 0.0, "Diagonal " + i + " must stay positive. " + f.describe());
   }

   /** The mean is the whole point of a hand-over: q, q̇ and the gyro biases all come from the outgoing estimator. */
   @Test
   public void testHandoverSeedTakesMeanFromModelAndBiasProvider()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(9999L, 4, 0, 3);
      f.filter.initialize();

      // Encoders deliberately disagree with the model, so a seed that fell back to raw encoders would fail.
      for (OneDoFJointBasics joint : f.filteredJoints)
         f.setEncoder(joint, -7.0);

      for (int i = 0; i < f.n; i++)
      {
         f.filteredJoints.get(i).setQ(0.11 * (i + 1));
         f.filteredJoints.get(i).setQd(-0.07 * (i + 1));
      }

      f.filter.seedFromHandover(new UniformBiasProvider(0.001, -0.002, 0.003));

      DMatrixRMaj x = f.filter.getStateVector();
      for (int i = 0; i < f.n; i++)
      {
         assertEquals(0.11 * (i + 1), x.get(i, 0), TOL, "q[" + i + "] must come from the model. " + f.describe());
         assertEquals(-0.07 * (i + 1), x.get(f.n + i, 0), TOL, "qd[" + i + "] must come from the model. " + f.describe());
      }
      for (int o = 0; o < f.m; o++)
      {
         int biasColumn = 2 * f.n + 3 * o;
         assertEquals(0.001, x.get(biasColumn, 0), TOL, "bias x of IMU " + o + ". " + f.describe());
         assertEquals(-0.002, x.get(biasColumn + 1, 0), TOL, "bias y of IMU " + o + ". " + f.describe());
         assertEquals(0.003, x.get(biasColumn + 2, 0), TOL, "bias z of IMU " + o + ". " + f.describe());
      }
   }

   /** A null bias source is legal (the outgoing estimator may not have one) and leaves the bias block zeroed. */
   @Test
   public void testHandoverSeedWithoutBiasSourceZeroesBiasBlock()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(2468L, 4, 0, 3);
      f.filter.initialize();

      f.filter.seedFromHandover(null);

      DMatrixRMaj x = f.filter.getStateVector();
      for (int i = 2 * f.n; i < f.dim; i++)
         assertEquals(0.0, x.get(i, 0), TOL, "Bias entry " + i + " must be zero without a bias source. " + f.describe());
   }

   /**
    * The hand-over must bypass the on-ground debounce that {@link JointLevelKFPreFilter#initialize()} honours.
    * That gate guards a boot with an unobservable bias; a hand-over is the opposite case.
    */
   @Test
   public void testHandoverSeedBypassesTheOnGroundGate()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(1357L, 4, 0, 3);
      f.filter.setInitializationGate(() -> false);

      f.filter.initialize();
      assertTrue(Double.isNaN(f.filter.getEstimatedJointPosition(f.filteredJoints.get(0))),
                 "Precondition: a shut gate must leave initialize() a no-op. " + f.describe());

      f.filter.seedFromHandover(null);

      assertFalse(Double.isNaN(f.filter.getEstimatedJointPosition(f.filteredJoints.get(0))),
                  "The hand-over seed must initialize the filter despite the shut gate. " + f.describe());
   }

   /** A KF permanently latches NaN, so a non-finite hand-over must be refused outright rather than seeded. */
   @Test
   public void testHandoverSeedRejectsNonFiniteState()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(8642L, 4, 0, 3);
      f.filter.initialize();
      DMatrixRMaj before = f.filter.getStateVector();

      f.filteredJoints.get(0).setQd(Double.NaN);
      f.filter.seedFromHandover(null);

      JointLevelKFTestFixture.assertAllClose(f.filter.getStateVector(), before, TOL,
                                             "A non-finite hand-over must leave the state untouched. " + f.describe());
   }

   private static boolean isAllClose(DMatrixRMaj a, DMatrixRMaj b, double tol)
   {
      for (int i = 0; i < a.getNumElements(); i++)
      {
         if (Math.abs(a.get(i) - b.get(i)) > tol)
            return false;
      }
      return true;
   }
}
