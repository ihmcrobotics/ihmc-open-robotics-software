package us.ihmc.stateEstimation.head;

import java.io.IOException;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.FilterTools.ProccessNoiseModel;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class EKFHeadPoseEstimatorTest
{
   private final static boolean VISUALIZE = false;
   private static final String PARAMETER_FILE = "headPoseEstimatorTest.xml";
   private static final Random RANDOM = new Random(58369L);

   @Test
   public void testSimpleTrajectory() throws IOException
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      FilterTools.proccessNoiseModel = ProccessNoiseModel.PIECEWISE_CONTINUOUS_ACCELERATION;

      double duration = 30.0;
      double timePerWaypoint = duration / 3.0;
      double dt = 0.004;

      // Create a simple joint structure with the head connected to a floating joint:
      RigidBodyTransform imuTransform = EuclidCoreRandomTools.nextRigidBodyTransform(RANDOM);
      RigidBodyBasics elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint headJoint = new SixDoFJoint("head_joint", elevator);
      RigidBodyBasics headBody = new RigidBody("imu_body", headJoint, 0.1, 0.1, 0.1, 1.0, new Vector3D());
      MovingReferenceFrame headFrame = headJoint.getFrameAfterJoint();
      MovingReferenceFrame imuFrame = MovingReferenceFrame.constructFrameFixedInParent("imu", headJoint.getFrameAfterJoint(), imuTransform);

      // Create a simulated IMU:
      SimulatedIMU simulatedIMU = new SimulatedIMU(imuFrame, headBody);
      YoFramePoseUsingYawPitchRoll headPose = new YoFramePoseUsingYawPitchRoll("HeadPose", ReferenceFrame.getWorldFrame(), registry);

      // Create a trajectory for the floating joint:
      int numberOfWaypoints = (int) (duration / timePerWaypoint) + 1;
      MultipleWaypointsPoseTrajectoryGenerator trajectoryGenerator = new MultipleWaypointsPoseTrajectoryGenerator("Trajectory", numberOfWaypoints, registry);
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         addRandomWaypoint(trajectoryGenerator, timePerWaypoint * i);
      }
      trajectoryGenerator.initialize();

      // Create an EKF based orientation estimator:
      EKFHeadPoseEstimator poseEstimator = new EKFHeadPoseEstimator(dt, imuFrame.getTransformToDesiredFrame(headFrame), true);
      registry.addChild(poseEstimator.getRegistry());
      XmlParameterReader reader = new XmlParameterReader(getClass().getResourceAsStream("/" + PARAMETER_FILE));
      Set<String> defaultParameters = new HashSet<>();
      Set<String> unmatchedParameters = new HashSet<>();
      reader.readParametersInRegistry(registry, defaultParameters, unmatchedParameters);
      unmatchedParameters.forEach(p -> System.out.println("Did not find parameter " + p));

      Vector3DBasics bias = new YoFrameVector3D("AngularVelocityBias", imuFrame, registry);

      // If we would like to plot some data create an instance of SCS:
      SimulationConstructionSet scs;
      if (VISUALIZE)
      {
         Robot robot = new Robot("Head");
         scs = new SimulationConstructionSet(robot);
         scs.getRootRegistry().addChild(registry);
      }

      // Simulate the head moving:
      double t = 0.0;
      boolean initialize = true;
      FrameVector3D zAxis = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
      FrameVector3D north = EuclidFrameRandomTools.nextOrthogonalFrameVector3D(RANDOM, zAxis, false);

      while (t < duration)
      {
         // At some point change the angular velocity bias to check if it estimated nicely:
         boolean biasChanging = false;
         if (t > duration / 4.0 && t < duration / 2.0)
         {
            bias.setX((t - duration / 4.0) * 0.01);
            biasChanging = true;
         }

         // Update the simulation:
         updateJoint(headJoint, trajectoryGenerator, t);
         headFrame.update();
         imuFrame.update();
         simulatedIMU.compute();
         headPose.setFromReferenceFrame(headFrame);

         // This is simulated clean data coming from the head IMU:
         FrameVector3DReadOnly imuAngularVelocityMeasurement = simulatedIMU.getAngularVelocity();
         FrameVector3DReadOnly imuLinearAccelerationMeasurement = simulatedIMU.getLinearAcceleration();

         // We can use an EKF based orientation estimator to estimate the IMU orientation:
         if (initialize)
         {
            poseEstimator.initialize(headFrame.getTransformToWorldFrame(), north);
            initialize = false;
         }
         FramePoint3D headPosition = new FramePoint3D(headFrame);
         headPosition.changeFrame(ReferenceFrame.getWorldFrame());
         FrameVector3D magneticFieldVector = new FrameVector3D(north);
         magneticFieldVector.changeFrame(imuFrame);
         FrameVector3D angularVelocityMeasurement = new FrameVector3D(imuAngularVelocityMeasurement);
         angularVelocityMeasurement.add(bias);

         poseEstimator.setImuAngularVelocity(angularVelocityMeasurement);
         poseEstimator.setImuLinearAcceleration(imuLinearAccelerationMeasurement);
         poseEstimator.setImuMagneticFieldVector(magneticFieldVector);
         poseEstimator.setEstimatedHeadPosition(headPosition);
         poseEstimator.compute();

         // Update SCS:
         if (VISUALIZE)
         {
            scs.setTime(t);
            scs.tickAndUpdate();
         }
         else
         {
            double translationEpsilon = 1.0e-5;
            double rotationEpsilon = biasChanging ? Math.toRadians(5.0) : Math.toRadians(1.0);

            RigidBodyTransform actual = new RigidBodyTransform();
            RigidBodyTransform expected = new RigidBodyTransform();
            poseEstimator.getHeadTransform(actual);
            headPose.get(expected);
            EuclidCoreTestTools.assertTuple3DEquals(expected.getTranslationVector(), actual.getTranslationVector(), translationEpsilon);
            EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected.getRotationMatrix(), actual.getRotationMatrix(), rotationEpsilon);
         }

         t += dt;
      }

      // Show SCS:
      if (VISUALIZE)
      {
         scs.setOutPoint();
         scs.cropBuffer();
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
   }

   private void updateJoint(SixDoFJoint headJoint, MultipleWaypointsPoseTrajectoryGenerator trajectoryGenerator, double t)
   {
      FrameQuaternion orientation = new FrameQuaternion();
      FrameVector3D angularVelocity = new FrameVector3D();
      FrameVector3D angularAcceleration = new FrameVector3D();
      FramePoint3D position = new FramePoint3D();
      FrameVector3D linearVelocity = new FrameVector3D();
      FrameVector3D linearAcceleration = new FrameVector3D();

      trajectoryGenerator.compute(t);
      trajectoryGenerator.getAngularData(orientation, angularVelocity, angularAcceleration);
      trajectoryGenerator.getLinearData(position, linearVelocity, linearAcceleration);

      MovingReferenceFrame bodyFrame = headJoint.getFrameAfterJoint();
      Twist jointTwist = new Twist(bodyFrame, ReferenceFrame.getWorldFrame(), angularVelocity, linearVelocity);
      SpatialAcceleration jointAcceleration = new SpatialAcceleration(bodyFrame, ReferenceFrame.getWorldFrame(), angularAcceleration, linearAcceleration);

      jointTwist.changeFrame(bodyFrame);
      jointAcceleration.changeFrame(bodyFrame);

      headJoint.setJointConfiguration(orientation, position);
      headJoint.setJointAngularVelocity(jointTwist.getAngularPart());
      headJoint.setJointLinearVelocity(jointTwist.getLinearPart());
      headJoint.setJointAngularAcceleration(jointAcceleration.getAngularPart());
      headJoint.setJointLinearAcceleration(jointAcceleration.getLinearPart());
      headJoint.updateFramesRecursively();
   }

   private void addRandomWaypoint(MultipleWaypointsPoseTrajectoryGenerator trajectoryGenerator, double time)
   {
      FramePose3D pose = EuclidFrameRandomTools.nextFramePose3D(RANDOM, ReferenceFrame.getWorldFrame());
      FrameVector3D linearVelocity = EuclidFrameRandomTools.nextFrameVector3D(RANDOM, ReferenceFrame.getWorldFrame());
      FrameVector3D angularVelocity = EuclidFrameRandomTools.nextFrameVector3D(RANDOM, ReferenceFrame.getWorldFrame());
      // Waypoint at time zero should have zero velocity.
      //      if (Precision.equals(time, 0.0))
      {
         angularVelocity.setToZero();
         linearVelocity.setToZero();
      }
      trajectoryGenerator.appendPoseWaypoint(time, pose, linearVelocity, angularVelocity);
   }
}
