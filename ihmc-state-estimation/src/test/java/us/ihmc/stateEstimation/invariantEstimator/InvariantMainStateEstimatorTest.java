package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import gnu.trove.map.hash.TObjectDoubleHashMap;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.FullRobotModelTestTools.RandomFullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
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
                                                 -9.81,  // gravitationalAcceleration
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

   @Test
   public void testContactAddRemoveStartsHangingThenAdmitsFeetOnContact()
   {
      // Robot starts hanging on the gantry: no ground contact. buildRig defaults to the ADD_REMOVE lifecycle.
      Rig rig = buildRig(4006L, new RotationMatrix(), 0.0, false);
      setGravityConsistentAccel(rig);
      for (SettableTestIMU imu : rig.imus)
         imu.setAngularVelocity(0.0, 0.0, 0.0);

      InvariantEKFStateEstimator estimator = rig.main.getInvariantEKFStateEstimator();
      assertTrue(estimator.isUsingContactAddRemoveMode(), "add/remove is the default mode");

      // While hanging, neither foot is admitted to the state.
      for (int k = 0; k < 200; k++)
         rig.doControl();
      assertFalse(estimator.isContactActive(RobotSide.LEFT), "left foot must stay out of the state while hanging");
      assertFalse(estimator.isContactActive(RobotSide.RIGHT), "right foot must stay out of the state while hanging");

      // Set the robot down: both feet come into solid contact and are augmented into the state.
      estimator.setContactProbabilityProvider(constantContact(1.0));
      for (int k = 0; k < 50; k++)
         rig.doControl();
      assertTrue(estimator.isContactActive(RobotSide.LEFT), "left foot admitted once it bears load");
      assertTrue(estimator.isContactActive(RobotSide.RIGHT), "right foot admitted once it bears load");
      assertPipelineFinite(rig, "hanging-then-contact add/remove");
   }

   // ---------------------------------------------------------------------------------------------------------

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
