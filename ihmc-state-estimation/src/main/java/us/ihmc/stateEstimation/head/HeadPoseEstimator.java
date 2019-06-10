package us.ihmc.stateEstimation.head;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearAccelerationSensor;
import us.ihmc.ekf.filter.sensor.implementations.MagneticFieldSensor;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class HeadPoseEstimator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PoseState poseState;
   private final LinearAccelerationSensor linearAccelerationSensor;
   private final MagneticFieldSensor magneticFieldSensor;
   private final AngularVelocitySensor angularVelocitySensor;
   private final PositionSensor positionSensor;
   private final StateEstimator stateEstimator;

   private final SixDoFJoint headJoint;

   private final RigidBodyTransform headTransform = new RigidBodyTransform();
   private final Twist headTwist = new Twist();

   private final YoFramePoseUsingYawPitchRoll headPose = new YoFramePoseUsingYawPitchRoll("EstimatedHeadPose", ReferenceFrame.getWorldFrame(), registry);

   public HeadPoseEstimator(double dt, RigidBodyTransform imuToHead, YoVariableRegistry parentRegistry)
   {
      // Creates a simple joint structure representing the head attached to world with a floating joint:
      RigidBodyBasics elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      headJoint = new SixDoFJoint("head_joint", elevator);
      RigidBodyBasics headBody = new RigidBody("imu_body", headJoint, 0.1, 0.1, 0.1, 1.0, new Vector3D());
      MovingReferenceFrame headFrame = headJoint.getFrameAfterJoint();
      MovingReferenceFrame imuFrame = MovingReferenceFrame.constructFrameFixedInParent("imu", headFrame, imuToHead);

      // Create all the sensors:
      angularVelocitySensor = new AngularVelocitySensor("AngularVelocity", dt, headBody, imuFrame, true, registry);
      linearAccelerationSensor = new LinearAccelerationSensor("LinearAcceleration", dt, headBody, imuFrame, false, registry);
      magneticFieldSensor = new MagneticFieldSensor("MagneticField", dt, headBody, imuFrame, registry);
      positionSensor = new PositionSensor("Position", dt, registry);
      List<Sensor> sensors = Arrays.asList(new Sensor[] {angularVelocitySensor, positionSensor, linearAccelerationSensor, magneticFieldSensor});

      // Create the state and the estimator:
      poseState = new PoseState("Head", dt, headFrame, registry);
      RobotState robotState = new RobotState(poseState, Collections.emptyList());
      stateEstimator = new StateEstimator(sensors, robotState, registry);

      headTwist.setToZero(headJoint.getFrameAfterJoint(), headJoint.getFrameBeforeJoint(), headJoint.getFrameAfterJoint());

      parentRegistry.addChild(registry);
   }

   public void initialize(RigidBodyTransform initialHeadTransform, FrameVector3D magneticFieldDirection)
   {
      poseState.initialize(initialHeadTransform, headTwist);
      magneticFieldSensor.setNorth(magneticFieldDirection);
      linearAccelerationSensor.resetBias();
      angularVelocitySensor.resetBias();
      updateRobot();
   }

   public void setImuAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      angularVelocitySensor.setMeasurement(angularVelocity);
   }

   public void setImuLinearAcceleration(Vector3DReadOnly linearAcceleration)
   {
      linearAccelerationSensor.setMeasurement(linearAcceleration);
   }

   public void setImuMagneticFieldVector(Vector3DReadOnly magneticFieldVector)
   {
      magneticFieldSensor.setMeasurement(magneticFieldVector);
   }

   public void setEstimatedHeadPosition(FramePoint3DReadOnly headPosition)
   {
      positionSensor.setMeasurement(headPosition);
   }

   public void compute()
   {
      poseState.getTransform(headTransform);
      headPose.set(headTransform);

      stateEstimator.predict();
      updateRobot();
      stateEstimator.correct();
      updateRobot();
   }

   private void updateRobot()
   {
      poseState.getTransform(headTransform);
      headJoint.setJointConfiguration(headTransform);
      poseState.getTwist(headTwist);
      headJoint.setJointTwist(headTwist);
      headJoint.updateFramesRecursively();
   }
}
