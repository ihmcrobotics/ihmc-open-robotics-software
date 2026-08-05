package us.ihmc.stateEstimation.invariant_estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.junit.jupiter.api.Test;

import gnu.trove.map.hash.TObjectDoubleHashMap;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.FullRobotModelTestTools.RandomFullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.jointLevel.JointLevelKFPreFilter;
import us.ihmc.stateEstimation.jointLevel.JointLevelKFTestSupport;
import us.ihmc.stateEstimation.jointLevel.SettableTestIMU;
import us.ihmc.stateEstimation.jointLevel.SettableTestSensorMap;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * "Full filter" tests of {@link InvariantMainStateEstimator} — the InEKF floating-base estimator wrapping a
 * {@link JointLevelKFPreFilter} pre-filter, which is what actually runs on the robot. Built on a synthetic
 * {@link RandomFullHumanoidRobotModel} + a hand-fed {@link SettableTestSensorMap}, with no SCS simulation.
 *
 * <p>Because the model geometry is random (not a realistic standing posture), every scenario is made
 * self-consistent by construction: a forced constant contact probability (so behavior does not depend on
 * where random feet land), and either a fully static equilibrium, a pure base rotation in no-contact hold,
 * or a gravity-consistent hang. These lock in the assembled-pipeline behavior and the NaN hardening.</p>
 */
public class InvariantMainStateEstimatorTest
{
   private static final double DT = 1.0e-3;
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();

   /** Everything a scenario needs to drive and read one assembled estimator. */
   private static final class Rig
   {
      RandomFullHumanoidRobotModel model;
      OneDoFJointBasics[] joints;
      SettableTestSensorMap sensorMap;
      SettableTestIMU pelvisIMU;
      List<SettableTestIMU> imus;
      JointLevelKFPreFilter preFilter;
      InvariantMainStateEstimator main;
      InvariantEKF ekf;

      void doControl()
      {
         main.doControl();
      }
   }

   /**
    * Assembles the model, sensors, pre-filter over both leg IMU pairs, and the main estimator; seeds the base
    * at {@code baseOrientation} and forces both feet to {@code contactProbability}. If {@code poisonLegImu} is
    * true, the left-foot IMU's bias covariance is corrupted before the pre-filter is built (NaN regression).
    */
   private static Rig buildRig(long seed, RotationMatrix baseOrientation, double contactProbability, boolean poisonLegImu)
   {
      Rig rig = new Rig();
      Random random = new Random(seed);
      rig.model = new RandomFullHumanoidRobotModel(random);
      rig.joints = rig.model.getOneDoFJoints();

      // A fixed, mild static posture; velocities zero. Geometry is random regardless of q.
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, rig.joints);
      for (OneDoFJointBasics joint : rig.joints)
         joint.setQd(0.0);
      rig.model.getRootJoint().setJointConfigurationToZero();
      rig.model.getElevator().updateFramesRecursively();

      rig.pelvisIMU = new SettableTestIMU("pelvisIMU", rig.model.getPelvis());
      SettableTestIMU leftFootIMU = new SettableTestIMU("leftFootIMU", rig.model.getFoot(RobotSide.LEFT));
      SettableTestIMU rightFootIMU = new SettableTestIMU("rightFootIMU", rig.model.getFoot(RobotSide.RIGHT));
      if (poisonLegImu)
         leftFootIMU.setBiasProcessNoiseCovarianceNonFinite();
      rig.imus = Arrays.asList(rig.pelvisIMU, leftFootIMU, rightFootIMU);
      List<IMUSensorReadOnly> imuOutputs = new ArrayList<>(rig.imus);

      rig.sensorMap = new SettableTestSensorMap(imuOutputs, Arrays.asList(rig.joints));
      for (OneDoFJointBasics joint : rig.joints)
         rig.sensorMap.setPosition(joint, joint.getQ()); // encoder = model q

      List<IMUBasedJointStateEstimatorParameters> pairParameters = new ArrayList<>();
      pairParameters.add(new IMUBasedJointStateEstimatorParameters("leftLeg", true, "pelvisIMU", "leftFootIMU", 0.0, 0.0));
      pairParameters.add(new IMUBasedJointStateEstimatorParameters("rightLeg", true, "pelvisIMU", "rightFootIMU", 0.0, 0.0));
      List<RigidBodyBasics> feet = Arrays.asList(rig.model.getFoot(RobotSide.LEFT), rig.model.getFoot(RobotSide.RIGHT));
      rig.preFilter = JointLevelKFTestSupport.newPreFilter(rig.sensorMap, pairParameters, feet, DT, new YoRegistry("preFilter"));

      rig.main = new InvariantMainStateEstimator(rig.model,
                                                 rig.sensorMap,
                                                 "pelvisIMU",
                                                 null,   // centerOfMassDataHolder — supported
                                                 DT,
                                                 1.0e-4, // gyroVariance
                                                 1.0e-3, // accelVariance
                                                 1.0e-6, // contactVariance
                                                 1.0e-6, // contactMeasurementVariance
                                                 1.0,    // initialCovariance
                                                 false,  // enableYawSeeding
                                                 rig.preFilter);
      rig.main.getInvariantEKFStateEstimator().setContactProbabilityProvider(constantContact(contactProbability));
      rig.ekf = rig.main.getInvariantEKFStateEstimator().getInvariantEKF();

      RigidBodyTransform baseTransform = new RigidBodyTransform();
      baseTransform.getRotation().set(baseOrientation);
      rig.main.initializeEstimator(baseTransform, new TObjectDoubleHashMap<>());
      return rig;
   }

   @Test
   public void testStaticEquilibriumStaysPutAndFinite()
   {
      Rig rig = buildRig(4001L, new RotationMatrix(), 1.0, false); // identity base, feet in contact
      setGravityConsistentAccel(rig);                              // stationary specific force
      for (SettableTestIMU imu : rig.imus)
         imu.setAngularVelocity(0.0, 0.0, 0.0);

      for (int k = 0; k < 1000; k++)
         rig.doControl();

      RotationMatrix rotation = new RotationMatrix();
      rig.ekf.getRotation(rotation);
      assertTrue(rotation.distance(new RotationMatrix()) < 1.0e-3, "base orientation stays at identity in static equilibrium");

      Vector3D velocity = new Vector3D();
      rig.ekf.getBaseVelocity(velocity);
      assertTrue(velocity.norm() < 1.0e-2, "base velocity stays near zero in static equilibrium: " + velocity.norm());

      // Leg-joint estimates track the fed encoders and are written onto the model.
      for (OneDoFJointBasics joint : rig.joints)
      {
         if (!rig.preFilter.containsJoint(joint))
            continue;
         double encoder = joint.getQ();
         assertEquals(encoder, rig.preFilter.getEstimatedJointPosition(joint), 5.0e-3, "leg-joint estimate tracks encoder: " + joint.getName());
      }

      assertPipelineFinite(rig, "static equilibrium");
   }

   @Test
   public void testNoContactRotationIntegrates()
   {
      Rig rig = buildRig(4002L, new RotationMatrix(), 0.0, false); // no contact → hold mode
      double rate = 0.2; // rad/s about world Y
      int ticks = 1000;   // 1.0 s
      for (int k = 0; k < ticks; k++)
      {
         setRigidRotationGyros(rig, 0.0, rate, 0.0);
         rig.doControl();
      }
      RotationMatrix rotation = new RotationMatrix();
      rig.ekf.getRotation(rotation);
      double sweptAngle = rotation.distance(new RotationMatrix());
      double expected = rate * ticks * DT;
      assertEquals(expected, sweptAngle, 2.0e-2, "no-contact orientation integrates the gyro (swept angle)");
      assertPipelineFinite(rig, "no-contact rotation");
   }

   @Test
   public void testHangingUnderGravityIsCompensated()
   {
      RotationMatrix tilt = new RotationMatrix();
      tilt.setToRollOrientation(Math.toRadians(20.0)); // hang tilted 20° about x
      Rig rig = buildRig(4003L, tilt, 0.0, false);     // no contact → hold mode
      setGravityConsistentAccel(rig);                  // specific force a hanging IMU reads at this tilt
      for (SettableTestIMU imu : rig.imus)
         imu.setAngularVelocity(0.0, 0.0, 0.0);

      Vector3D startPosition = new Vector3D();
      rig.ekf.getBasePosition(startPosition);

      for (int k = 0; k < 1000; k++)
         rig.doControl();

      RotationMatrix rotation = new RotationMatrix();
      rig.ekf.getRotation(rotation);
      assertTrue(rotation.distance(tilt) < 1.0e-3, "orientation stays at the hang tilt (gyro is zero)");

      Vector3D velocity = new Vector3D();
      rig.ekf.getBaseVelocity(velocity);
      assertTrue(velocity.norm() < 1.0e-3, "base velocity held near zero while hanging: " + velocity.norm());

      Vector3D endPosition = new Vector3D();
      rig.ekf.getBasePosition(endPosition);
      endPosition.sub(startPosition);
      assertTrue(endPosition.norm() < 1.0e-3, "gravity compensation keeps the hanging base in place (drift " + endPosition.norm() + ")");

      assertPipelineFinite(rig, "hanging under gravity");
   }

   @Test
   public void testFreeFallDoesNotRunAwayOrFail()
   {
      Rig rig = buildRig(4004L, new RotationMatrix(), 0.0, false); // no contact → hold mode
      for (SettableTestIMU imu : rig.imus)
      {
         imu.setAngularVelocity(0.0, 0.0, 0.0);
         imu.setLinearAcceleration(0.0, 0.0, 0.0); // weightless: zero specific force
      }
      Vector3D startPosition = new Vector3D();
      rig.ekf.getBasePosition(startPosition);

      for (int k = 0; k < 2000; k++)
      {
         rig.doControl();
         Vector3D velocity = new Vector3D();
         rig.ekf.getBaseVelocity(velocity);
         assertTrue(velocity.norm() < 1.0e-6, "held velocity does not ramp in free fall: " + velocity.norm());
      }
      Vector3D endPosition = new Vector3D();
      rig.ekf.getBasePosition(endPosition);
      endPosition.sub(startPosition);
      assertTrue(endPosition.norm() < 2.0e-2, "free-fall base position drift stays bounded (no runaway): " + endPosition.norm());
      assertPipelineFinite(rig, "free fall");
   }

   @Test
   public void testPoisonedLegImuDoesNotFailPipeline()
   {
      Rig rig = buildRig(4005L, new RotationMatrix(), 1.0, true); // left-foot IMU covariance poisoned before build
      setGravityConsistentAccel(rig);
      for (SettableTestIMU imu : rig.imus)
         imu.setAngularVelocity(0.0, 0.0, 0.0);

      for (int k = 0; k < 500; k++)
         rig.doControl(); // must not throw (a NaN bias would blow up ekf.predict)

      RotationMatrix rotation = new RotationMatrix();
      rig.ekf.getRotation(rotation);
      assertTrue(Double.isFinite(rotation.distance(new RotationMatrix())), "InEKF orientation finite despite a poisoned leg IMU");
      assertPipelineFinite(rig, "poisoned leg IMU");
   }

   /**
    * Gauge-shift correctness for {@link InvariantMainStateEstimator#requestReinitializeEstimatorToWorldOrigin()}.
    *
    * <p>The InEKF state carries the base position p and every contact anchor p_c,i as absolute world
    * positions, and the contact update drives the residual y_i = R^T (p_c,i - p) to its FK prediction. A
    * reset that moved p without moving each p_c,i by the same delta would inject a residual of -R^T delta on
    * BOTH feet, which the filter would then "correct" by snapping the base back. So the load-bearing
    * assertion here is not "the base moved" but "the filter's residual still agrees with forward
    * kinematics" — which is exactly what it means for p and every p_c,i to have moved together.</p>
    */
   @Test
   public void testReinitializeToWorldOriginIsAGaugeShift()
   {
      Rig rig = staticRigAtBasePosition(4006L, new Vector3D(1.7, -0.9, 0.35));

      Vector3D midFeetBefore = midFeetInWorld(rig);
      assertTrue(midFeetBefore.norm() > 1.0e-2,
                 "precondition: the reset must have a non-trivial translation to make (" + midFeetBefore.norm() + ")");

      RotationMatrix rotationBefore = new RotationMatrix();
      rig.ekf.getRotation(rotationBefore);
      SideDependentList<Vector3D> fkResidualBefore = new SideDependentList<>(forwardKinematicsContactResidual(rig, RobotSide.LEFT),
                                                                            forwardKinematicsContactResidual(rig, RobotSide.RIGHT));

      rig.main.requestReinitializeEstimatorToWorldOrigin();
      rig.doControl();

      // 1. The gauge invariant: the filter's contact residual still matches the model's forward kinematics,
      //    i.e. the anchors travelled with the base. This is the assertion that fails loudly (by |delta|,
      //    ~2 m here) if someone "simplifies" the reset into a direct write of the base position.
      for (RobotSide side : RobotSide.values)
      {
         Vector3D mismatch = filterContactResidual(rig, side);
         mismatch.sub(forwardKinematicsContactResidual(rig, side));
         assertTrue(mismatch.norm() < 1.0e-6, "contact anchors moved with the base (" + side + " residual mismatch " + mismatch.norm() + ")");
      }
      // ...and the FK residual itself is untouched, because a rigid translation cannot change body-frame geometry.
      for (RobotSide side : RobotSide.values)
      {
         Vector3D fkDrift = forwardKinematicsContactResidual(rig, side);
         fkDrift.sub(fkResidualBefore.get(side));
         assertTrue(fkDrift.norm() < 1.0e-9, "translation does not change the body-frame foot geometry (" + side + ")");
      }

      // 2. Mid-feet lands on the world origin.
      assertTrue(midFeetInWorld(rig).norm() < 1.0e-5, "mid-feet at the world origin after the reset: " + midFeetInWorld(rig).norm());

      // 3. Translation only: orientation, and therefore yaw, is untouched.
      RotationMatrix rotationAfter = new RotationMatrix();
      rig.ekf.getRotation(rotationAfter);
      assertTrue(rotationAfter.distance(rotationBefore) < 1.0e-6, "the reset is translation only: orientation unchanged");
      assertEquals(rotationBefore.getYaw(), rotationAfter.getYaw(), 1.0e-6, "yaw unchanged by the reset");

      // 4. The covariance is still a covariance.
      assertCovarianceSymmetricAndPSD(rig, "reinitialize to world origin");

      // 5. No snap-back: a consistent gauge shift leaves the contact update nothing to correct.
      for (int k = 0; k < 500; k++)
         rig.doControl();
      assertTrue(midFeetInWorld(rig).norm() < 1.0e-2, "base does not snap back after the reset: " + midFeetInWorld(rig).norm());
      assertPipelineFinite(rig, "after reinitialize to world origin");
   }

   /** The plain reinit re-seeds in place: anchors and covariance are refreshed, the estimate does not move. */
   @Test
   public void testReinitializeInPlaceDoesNotMoveTheBase()
   {
      Rig rig = staticRigAtBasePosition(4007L, new Vector3D(1.7, -0.9, 0.35));

      Vector3D positionBefore = new Vector3D();
      rig.ekf.getBasePosition(positionBefore);

      rig.main.requestReinitializeEstimator();
      rig.doControl();

      Vector3D positionDrift = new Vector3D();
      rig.ekf.getBasePosition(positionDrift);
      positionDrift.sub(positionBefore);
      assertTrue(positionDrift.norm() < 1.0e-6, "the plain reinit re-seeds in place: " + positionDrift.norm());

      for (RobotSide side : RobotSide.values)
      {
         Vector3D mismatch = filterContactResidual(rig, side);
         mismatch.sub(forwardKinematicsContactResidual(rig, side));
         assertTrue(mismatch.norm() < 1.0e-6, "anchors re-seeded from FK (" + side + ")");
      }
      assertCovarianceSymmetricAndPSD(rig, "reinitialize in place");
   }

   /**
    * The Z half of the reset is a discontinuity, and {@link KinematicContactDetector} finite-differences sole
    * height — so without {@link ContactProbabilityProvider#reset()} it reads the jump as a vertical speed of
    * dz/dt and mutes both feet. A/B on the same frames and the same jump: only the reset detector survives.
    */
   @Test
   public void testContactDetectorResetSuppressesTheResetJumpSpike()
   {
      Rig rig = buildRig(4008L, new RotationMatrix(), 1.0, false);
      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         soleFrames.put(side, rig.model.getSoleFrame(side));

      // Height gate deliberately wide open (h0 = 10 m, w = 1 m) so random model geometry cannot decide the
      // outcome: this test is about the SPEED gate, which is the one the jump trips. No smoothing, so the
      // spike shows up on the tick it happens.
      KinematicContactDetector withoutReset = new KinematicContactDetector(soleFrames, null, DT, 0.0, 10.0, 1.0, 0.50, 0.10, 0.0);
      KinematicContactDetector withReset = new KinematicContactDetector(soleFrames, null, DT, 0.0, 10.0, 1.0, 0.50, 0.10, 0.0);

      for (int k = 0; k < 3; k++) // settle: both feet firmly "in contact"
      {
         withoutReset.update();
         withReset.update();
      }
      for (RobotSide side : RobotSide.values)
         assertTrue(withReset.getContactProbability(side) > 0.99, "precondition: both feet read contact before the jump (" + side + ")");

      // The reset's translation, applied to the model exactly as reinitializeToMidFeetOrigin() would.
      Vector3D jumpedBasePosition = new Vector3D(rig.model.getRootJoint().getJointPose().getPosition());
      jumpedBasePosition.addZ(0.05);
      rig.model.getRootJoint().setJointPosition(jumpedBasePosition);
      rig.model.getElevator().updateFramesRecursively();

      withoutReset.update();
      withReset.reset();
      withReset.update();

      for (RobotSide side : RobotSide.values)
      {
         assertTrue(withoutReset.getContactProbability(side) < 0.01,
                    "without the reset the jump reads as 50 m/s and mutes the foot (" + side + "): " + withoutReset.getContactProbability(side));
         assertTrue(withReset.getContactProbability(side) > 0.99,
                    "reset() discards the history so the jump is not seen as motion (" + side + "): " + withReset.getContactProbability(side));
      }
   }

   // ---------------------------------------------------------------------------------------------------------

   /** A rig re-seeded with its base at {@code basePosition}, sensors set for a stationary robot. */
   private static Rig staticRigAtBasePosition(long seed, Vector3D basePosition)
   {
      Rig rig = buildRig(seed, new RotationMatrix(), 1.0, false);

      RigidBodyTransform baseTransform = new RigidBodyTransform();
      baseTransform.getTranslation().set(basePosition);
      rig.main.initializeEstimator(baseTransform, new TObjectDoubleHashMap<>());

      setGravityConsistentAccel(rig);
      for (SettableTestIMU imu : rig.imus)
         imu.setAngularVelocity(0.0, 0.0, 0.0);
      return rig;
   }

   /** The average sole-origin position in world — the point the reset drives to the origin. */
   private static Vector3D midFeetInWorld(Rig rig)
   {
      Vector3D midFeet = new Vector3D();
      for (RobotSide side : RobotSide.values)
      {
         FramePoint3D sole = new FramePoint3D(rig.model.getSoleFrame(side));
         sole.changeFrame(WORLD);
         midFeet.add(sole);
      }
      midFeet.scale(1.0 / RobotSide.values.length);
      return midFeet;
   }

   /** y_i = R^T (p_c,i - p), read straight out of the filter state. */
   private static Vector3D filterContactResidual(Rig rig, RobotSide side)
   {
      Vector3D basePosition = new Vector3D();
      rig.ekf.getBasePosition(basePosition);
      Vector3D residual = new Vector3D();
      rig.ekf.getContactPosition(side.ordinal(), residual); // CONTACT_INDICES: LEFT=0, RIGHT=1
      residual.sub(basePosition);

      RotationMatrix rotation = new RotationMatrix();
      rig.ekf.getRotation(rotation);
      rotation.inverseTransform(residual);
      return residual;
   }

   /** The same quantity predicted by the model: the sole origin expressed in the pelvis frame. */
   private static Vector3D forwardKinematicsContactResidual(Rig rig, RobotSide side)
   {
      FramePoint3D sole = new FramePoint3D(rig.model.getSoleFrame(side));
      sole.changeFrame(rig.model.getPelvis().getParentJoint().getFrameAfterJoint());
      return new Vector3D(sole);
   }

   private static void assertCovarianceSymmetricAndPSD(Rig rig, String scenario)
   {
      DMatrixRMaj covariance = rig.ekf.getState().getCovariance().copy();
      for (int r = 0; r < covariance.numRows; r++)
         for (int c = r + 1; c < covariance.numCols; c++)
            assertEquals(covariance.get(r, c), covariance.get(c, r), 1.0e-9, "covariance symmetric (" + scenario + ")");

      EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(covariance.numRows, false, true);
      assertTrue(eig.decompose(covariance), "covariance eigendecomposition succeeds (" + scenario + ")");
      for (int i = 0; i < eig.getNumberOfEigenvalues(); i++)
         assertTrue(eig.getEigenvalue(i).getReal() > -1.0e-9,
                    "covariance PSD (" + scenario + "): eigenvalue " + eig.getEigenvalue(i).getReal());
   }

   /** Sets every IMU's specific force to the true stationary reading at its current orientation: g_up in world. */
   private static void setGravityConsistentAccel(Rig rig)
   {
      for (SettableTestIMU imu : rig.imus)
      {
         FrameVector3D specificForce = new FrameVector3D(WORLD, 0.0, 0.0, 9.81);
         specificForce.changeFrame(imu.getMeasurementFrame());
         imu.setLinearAcceleration(specificForce);
      }
   }

   /** Sets every IMU's gyro to the body-frame projection of a common world angular velocity (rigid rotation). */
   private static void setRigidRotationGyros(Rig rig, double wx, double wy, double wz)
   {
      for (SettableTestIMU imu : rig.imus)
      {
         FrameVector3D omega = new FrameVector3D(WORLD, wx, wy, wz);
         omega.changeFrame(imu.getMeasurementFrame());
         imu.setAngularVelocity(omega.getX(), omega.getY(), omega.getZ());
      }
   }

   private static void assertPipelineFinite(Rig rig, String scenario)
   {
      RotationMatrix rotation = new RotationMatrix();
      rig.ekf.getRotation(rotation);
      Vector3D velocity = new Vector3D();
      rig.ekf.getBaseVelocity(velocity);
      Vector3D position = new Vector3D();
      rig.ekf.getBasePosition(position);
      assertTrue(Double.isFinite(rotation.distance(new RotationMatrix())) && velocity.norm() >= 0.0 && Double.isFinite(position.norm()),
                 "InEKF base estimate finite (" + scenario + ")");

      DMatrixRMaj covariance = rig.ekf.getState().getCovariance();
      for (int i = 0; i < covariance.getNumElements(); i++)
         assertTrue(Double.isFinite(covariance.get(i)), "InEKF covariance finite (" + scenario + ") at index " + i);
      for (int r = 0; r < covariance.numRows; r++)
         for (int c = r + 1; c < covariance.numCols; c++)
            assertTrue(Math.abs(covariance.get(r, c) - covariance.get(c, r)) < 1.0e-6, "InEKF covariance symmetric (" + scenario + ")");
   }

   private static ContactProbabilityProvider constantContact(double probability)
   {
      return new ContactProbabilityProvider()
      {
         @Override
         public void update()
         {
         }

         @Override
         public double getContactProbability(RobotSide side)
         {
            return probability;
         }
      };
   }
}
