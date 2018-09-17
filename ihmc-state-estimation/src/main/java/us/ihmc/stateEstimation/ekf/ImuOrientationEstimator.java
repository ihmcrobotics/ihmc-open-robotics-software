package us.ihmc.stateEstimation.ekf;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

/**
 * This class provides the functionality to estimate an IMU orientation given it's rate and acceleration measurements.
 * <p>
 * It can serve as a simple example of how to set up an estimator for a simple application using this library.
 * </p>
 */
public class ImuOrientationEstimator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PoseState poseState;
   private final LinearAccelerationSensor linearAccelerationSensor;
   private final AngularVelocitySensor angularVelocitySensor;

   private final StateEstimator stateEstimator;

   private final FrameQuaternion orientation = new FrameQuaternion();
   private final RigidBodyTransform imuTransform = new RigidBodyTransform();
   private final Twist imuTwist = new Twist();

   private final SixDoFJoint imuJoint;

   private final YoDouble orientationEstimationTime = new YoDouble("OrientationEstimationTime", registry);
   private final YoFrameYawPitchRoll yoOrientation = new YoFrameYawPitchRoll("EKFOrientation", ReferenceFrame.getWorldFrame(), registry);

   public ImuOrientationEstimator(double dt, YoVariableRegistry parentRegistry)
   {
      this(dt, false, false, parentRegistry);
   }

   public ImuOrientationEstimator(double dt, boolean estimateAngularVelocityBias, boolean estimateLiearAccelerationBias, YoVariableRegistry parentRegistry)
   {
      // Recreate a "dummy" inverse dynamics structure for the estimator:
      RigidBody elevator = new RigidBody("Elevator", ReferenceFrame.getWorldFrame());
      imuJoint = new SixDoFJoint("ImuJoint", elevator);
      RigidBody imuBody = ScrewTools.addRigidBody("ImuBody", imuJoint, 0.1, 0.1, 0.1, 1.0, new Vector3D());
      MovingReferenceFrame imuFrame = imuJoint.getFrameAfterJoint();

      // Create all the sensors:
      angularVelocitySensor = new AngularVelocitySensor("AngularVelocity", dt, imuBody, imuFrame, estimateAngularVelocityBias, registry);
      LinearVelocitySensor linearVelocitySensor = new LinearVelocitySensor("LinearVelocity", dt, imuBody, imuFrame, false, registry);
      linearAccelerationSensor = new LinearAccelerationSensor("LinearAcceleration", dt, imuBody, imuFrame, estimateLiearAccelerationBias, registry);
      List<Sensor> sensors = Arrays.asList(new Sensor[] {angularVelocitySensor, linearVelocitySensor, linearAccelerationSensor});

      // Create the state:
      poseState = new PoseState("ImuBody", dt, imuFrame, registry);
      RobotState robotState = new RobotState(poseState, Collections.emptyList());

      stateEstimator = new StateEstimator(sensors, robotState, registry);

      parentRegistry.addChild(registry);
   }

   public FrameQuaternionReadOnly update(Vector3DReadOnly angularVelocity, Vector3DReadOnly linearAcceleration)
   {
      long startTime = System.nanoTime();

      linearAccelerationSensor.setMeasurement(linearAcceleration);
      angularVelocitySensor.setMeasurement(angularVelocity);

      stateEstimator.predict();
      updateRobot();
      stateEstimator.correct();
      updateRobot();

      orientationEstimationTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));

      poseState.getOrientation(orientation);
      yoOrientation.set(orientation);
      return orientation;
   }

   private void updateRobot()
   {
      poseState.getTransform(imuTransform);
      imuJoint.setPositionAndRotation(imuTransform);
      poseState.getTwist(imuTwist);
      imuJoint.setJointTwist(imuTwist);
      imuJoint.updateFramesRecursively();
   }
}
