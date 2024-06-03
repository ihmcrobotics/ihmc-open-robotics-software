package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.robotics.math.filters.IntegratorBiasCompensatorYoFrameVector3D;
import us.ihmc.robotics.math.filters.YoIMUMahonyFilter;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBasedPelvisRotationalStateUpdater;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This estimator takes advantage of having several IMUs distributed on the robot to refine the pose
 * estimate of the robot parts.
 * <p>
 * The estimated CoM state relies on accurate modeling of the robot bodies.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class DistributedIMUBasedCenterOfMassStateUpdater implements MomentumStateUpdater
{
   private static final boolean MORE_VARIABLES = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble linearVelocityKp, linearVelocityKi;
   private final YoDouble positionKp, positionKi;
   private final YoDouble orientationKp, orientationKi;
   private final double dt;

   private final RigidBodyStateEstimator rootEstimator;
   private final RigidBodyStateEstimator[] estimators;
   private final Map<RigidBodyReadOnly, RigidBodyStateEstimator> estimatorMap;

   private final FrameVector3D gravityVector = new FrameVector3D();

   private final YoFramePoint3D estimatedCoMPosition = new YoFramePoint3D("estimatedCenterOfMassPosition", worldFrame, registry);
   private final YoFrameVector3D estimatedCoMVelocity = new YoFrameVector3D("estimatedCenterOfMassVelocity", worldFrame, registry);
   private final YoBoolean enableAdjustment = new YoBoolean("enableAdjustment", registry);
   private final YoFrameVector3D positionAdjustment = new YoFrameVector3D("estimatedCenterOfMassPositionAdjustment", worldFrame, registry);
   private final YoFrameVector3D velocityAdjustment = new YoFrameVector3D("estimatedCenterOfMassVelocityAdjustment", worldFrame, registry);

   private final CenterOfMassJacobian centerOfMassJacobian;
   private final YoFramePoint3D rawCoMPosition;
   private final YoFrameVector3D rawCoMVelocity;
   private final CenterOfMassDataHolder centerOfMassDataHolder;

   private final YoBoolean initialized = new YoBoolean("distIMUCoMEstimatorInitialized", registry);
   private final YoBoolean enableOutput = new YoBoolean("distIMUCoMEstimatorEnableOutput", registry);
   private final double gravitationalAcceleration;
   private final List<? extends RigidBodyReadOnly> listOfTrustedFeet;

   public DistributedIMUBasedCenterOfMassStateUpdater(FloatingJointReadOnly rootJoint,
                                                      List<? extends IMUSensorReadOnly> imuSensors,
                                                      List<? extends RigidBodyReadOnly> listOfTrustedFeet,
                                                      double dt,
                                                      double gravitationalAcceleration,
                                                      CenterOfMassDataHolder centerOfMassDataHolder)
   {
      this.listOfTrustedFeet = listOfTrustedFeet;
      this.dt = dt;
      this.gravitationalAcceleration = gravitationalAcceleration;
      this.centerOfMassDataHolder = centerOfMassDataHolder;

      linearVelocityKp = new YoDouble("linearVelocityKp", registry);
      linearVelocityKi = new YoDouble("linearVelocityKi", registry);
      positionKp = new YoDouble("positionKp", registry);
      positionKi = new YoDouble("positionKi", registry);
      orientationKp = new YoDouble("orientationKp", registry);
      orientationKi = new YoDouble("orientationKi", registry);

      linearVelocityKp.set(0.05);
      linearVelocityKi.set(0.005);
      positionKp.set(0.05);
      positionKi.set(0.005);
      orientationKp.set(0.05);
      orientationKi.set(0.005);
      enableOutput.set(true); // Appears to help Nadia's oscillations due to the fused spine and shaky arms.

      gravityVector.setIncludingFrame(worldFrame, 0.0, 0.0, -Math.abs(gravitationalAcceleration));

      Map<RigidBodyBasics, IMUSensorReadOnly> imuSensorMap = new HashMap<>();

      for (IMUSensorReadOnly imuSensor : imuSensors)
      {
         RigidBodyBasics body = imuSensor.getMeasurementLink();
         if (imuSensorMap.containsKey(body))
         {
            LogTools.warn("Already have an IMU for {}, ignoring IMU: {}", body, imuSensor.getSensorName());
            continue;
         }

         if (body.getName().toLowerCase().contains("foot"))
         {
            LogTools.warn("Ignoring IMU: {}", imuSensor.getSensorName());
            continue;
         }

         imuSensorMap.put(body, imuSensor);
      }

      rootEstimator = new RigidBodyStateEstimator(null, rootJoint.getSuccessor(), null);
      estimators = new RigidBodyStateEstimator[rootJoint.getSuccessor().subtreeArray().length];
      estimators[0] = rootEstimator;
      buildEstimatorsRecursive(0, rootEstimator, imuSensorMap);
      estimatorMap = Arrays.stream(estimators).collect(Collectors.toMap(estimator -> estimator.rigidBody, Function.identity()));

      if (MORE_VARIABLES)
      {
         centerOfMassJacobian = new CenterOfMassJacobian(rootJoint.getPredecessor(), worldFrame);
         rawCoMPosition = new YoFramePoint3D("rawCenterOfMassPosition", worldFrame, registry);
         rawCoMVelocity = new YoFrameVector3D("rawCenterOfMassVelocity", worldFrame, registry);
      }
      else
      {
         centerOfMassJacobian = null;
         rawCoMPosition = null;
         rawCoMVelocity = null;
      }
   }

   private int buildEstimatorsRecursive(int index, RigidBodyStateEstimator parent, Map<RigidBodyBasics, IMUSensorReadOnly> imuSensorMap)
   {
      for (JointReadOnly joint : parent.rigidBody.getChildrenJoints())
      {
         index++;
         RigidBodyStateEstimator child = new RigidBodyStateEstimator(parent, joint.getSuccessor(), imuSensorMap.get(joint.getSuccessor()));
         estimators[index] = child;
         index = buildEstimatorsRecursive(index, child, imuSensorMap);
      }

      return index;
   }

   @Override
   public void initialize()
   {
   }

   private boolean initializeInternal()
   {
      if (initialized.getValue())
         return false;

      for (RigidBodyStateEstimator estimator : estimators)
      {
         estimator.initialize();
      }
      initialized.set(true);
      return true;
   }

   @Override
   public void update()
   {
      if (initializeInternal())
         return;

      for (RigidBodyStateEstimator estimator : estimators)
      {
         estimator.updateState();
      }

      updateCoMState();
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final Momentum tempMomentum = new Momentum(worldFrame);

   private void updateCoMState()
   {
      double totalMass = 0.0;
      estimatedCoMPosition.setToZero();
      estimatedCoMVelocity.setToZero();

      for (RigidBodyStateEstimator estimator : estimators)
      {
         SpatialInertiaReadOnly inertia = estimator.rigidBody.getInertia();
         double mass = inertia.getMass();

         tempPoint.setIncludingFrame(inertia.getCenterOfMassOffset());
         if (estimator.isRoot())
         {
            tempPoint.changeFrame(worldFrame);
         }
         else
         {
            // We don't want to use tempPoint.changeFrame(worldFrame) as this would only use the kinematics data.
            tempPoint.applyTransform(estimator.getEstimatedPose());
            tempPoint.setReferenceFrame(worldFrame);
         }
         tempPoint.scale(mass);
         estimatedCoMPosition.add(tempPoint);

         tempMomentum.setReferenceFrame(inertia.getReferenceFrame());
         if (estimator.isRoot())
         {
            tempMomentum.compute(inertia, estimator.rigidBody.getBodyFixedFrame().getTwistOfFrame());
            tempVector.setIncludingFrame(tempMomentum.getLinearPart());
            tempVector.changeFrame(worldFrame);
         }
         else
         {
            tempMomentum.compute(inertia, estimator.getEstimatedTwist());
            tempVector.setIncludingFrame(tempMomentum.getLinearPart());
            // We don't want to use tempMomentum.changeFrame(worldFrame) as this would only use the kinematics data.
            tempVector.applyTransform(estimator.getEstimatedPose());
            tempVector.setReferenceFrame(worldFrame);
         }
         estimatedCoMVelocity.add(tempVector);

         totalMass += mass;
      }

      estimatedCoMPosition.scale(1.0 / totalMass);
      estimatedCoMVelocity.scale(1.0 / totalMass);

      { // Adjust using trusted feet
         double scale = 1.0 / listOfTrustedFeet.size();
         positionAdjustment.setToZero();
         velocityAdjustment.setToZero();

         for (int i = 0; i < listOfTrustedFeet.size(); i++)
         {
            RigidBodyReadOnly trutedFoot = listOfTrustedFeet.get(i);
            RigidBodyStateEstimator footEstimator = estimatorMap.get(trutedFoot);

            tempPoint.sub(footEstimator.getBodyFrame().getTransformToRoot().getTranslation(), footEstimator.estimatedPose.getTranslation());
            tempPoint.scale(scale);
            positionAdjustment.add(tempPoint);

            tempVector.setIncludingFrame(footEstimator.getEstimatedTwist().getLinearPart());
            tempVector.sub(footEstimator.getBodyFrame().getTwistOfFrame().getLinearPart());
            tempVector.changeFrame(worldFrame);
            tempVector.scale(scale);
            velocityAdjustment.add(tempVector);
         }

         if (enableAdjustment.getValue())
         {
            estimatedCoMPosition.add(positionAdjustment);
            velocityAdjustment.add(velocityAdjustment);
         }
      }

      if (enableOutput.getValue())
      {
         centerOfMassDataHolder.setCenterOfMassPosition(estimatedCoMPosition);
         centerOfMassDataHolder.setCenterOfMassVelocity(estimatedCoMVelocity);
      }
      else
      {
         centerOfMassDataHolder.clear();
      }

      if (MORE_VARIABLES)
      {
         centerOfMassJacobian.reset();
         rawCoMPosition.set(centerOfMassJacobian.getCenterOfMass());
         rawCoMVelocity.set(centerOfMassJacobian.getCenterOfMassVelocity());
      }
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   private class RigidBodyStateEstimator
   {
      private final RigidBodyStateEstimator parent;
      private final RigidBodyReadOnly rigidBody;
      private final IMUSensorReadOnly imuSensor;

      private final IntegratorBiasCompensatorYoFrameVector3D linearVelocityEstimate;
      private final IntegratorBiasCompensatorYoFrameVector3D positionEstimate;
      private final YoIMUMahonyFilter orientationFilter;

      private final YoFrameVector3D expectedStaticAcceleration;

      private final YoFramePose3D yoEstimatedPose;
      private final YoFixedFrameTwist yoEstimatedTwist;

      private final YoFramePose3D yoRawPose;
      private final YoFixedFrameTwist yoRawTwist;

      private final RigidBodyTransform estimatedPose = new RigidBodyTransform();
      private final Twist estimatedTwist = new Twist();
      private final Twist twistToParent = new Twist();

      public RigidBodyStateEstimator(RigidBodyStateEstimator parent, RigidBodyReadOnly rigidBody, IMUSensorReadOnly imuSensor)
      {
         this.parent = parent;
         this.rigidBody = rigidBody;
         this.imuSensor = imuSensor;

         if (parent == null)
         {
            yoEstimatedPose = null;
            yoEstimatedTwist = null;
            linearVelocityEstimate = null;
            positionEstimate = null;
            orientationFilter = null;
            expectedStaticAcceleration = null;
         }
         else
         {
            yoEstimatedPose = new YoFramePose3D(rigidBody.getName() + "Estimated", worldFrame, registry);
            yoEstimatedTwist = new YoFixedFrameTwist(getBodyFrame(),
                                                     worldFrame,
                                                     new YoFrameVector3D(rigidBody.getName() + "EstimatedAngularVelocity", getBodyFrame(), registry),
                                                     new YoFrameVector3D(rigidBody.getName() + "EstimatedLinearVelocity", getBodyFrame(), registry));

            if (imuSensor != null)
            {
               linearVelocityEstimate = new IntegratorBiasCompensatorYoFrameVector3D(rigidBody.getName()
                     + "LinearVelocityEstimate", registry, linearVelocityKp, linearVelocityKi, getIMUFrame(), dt);
               positionEstimate = new IntegratorBiasCompensatorYoFrameVector3D(rigidBody.getName()
                     + "PositionEstimate", registry, positionKp, positionKi, worldFrame, getBodyFrame(), dt);

               orientationFilter = new YoIMUMahonyFilter(rigidBody.getName(),
                                                         rigidBody.getName() + "OrientationEstimate",
                                                         "",
                                                         dt,
                                                         false,
                                                         getIMUFrame(),
                                                         orientationKp,
                                                         orientationKi,
                                                         registry);
               orientationFilter.setGravityMagnitude(gravitationalAcceleration);
               expectedStaticAcceleration = new YoFrameVector3D(rigidBody.getName() + "ExpectedStaticAcceleration", getIMUFrame(), registry);
            }
            else
            {
               linearVelocityEstimate = null;
               positionEstimate = null;
               orientationFilter = null;
               expectedStaticAcceleration = null;
            }
         }

         if (MORE_VARIABLES)
         {
            yoRawPose = new YoFramePose3D(rigidBody.getName() + "Raw", worldFrame, registry);
            yoRawTwist = new YoFixedFrameTwist(getBodyFrame(),
                                               worldFrame,
                                               new YoFrameVector3D(rigidBody.getName() + "RawAngularVelocity", getBodyFrame(), registry),
                                               new YoFrameVector3D(rigidBody.getName() + "RawLinearVelocity", getBodyFrame(), registry));
         }
         else
         {
            yoRawPose = null;
            yoRawTwist = null;
         }
      }

      public void initialize()
      {
         if (parent == null)
            return;

         if (imuSensor != null)
         {
            linearVelocityEstimate.reset();
            positionEstimate.reset();
            orientationFilter.initialize(getIMUFrame().getTransformToRoot().getRotation());
         }

         estimatedPose.set(getBodyFrame().getTransformToRoot());
         estimatedTwist.setToZero();
         yoEstimatedPose.set(getBodyFrame().getTransformToRoot());
         yoEstimatedTwist.setToZero();

         if (MORE_VARIABLES)
         {
            yoRawPose.set(getBodyFrame().getTransformToRoot());
            yoRawTwist.setToZero();
         }
      }

      private final FrameVector3D linearAcceleration = new FrameVector3D();
      private final FrameVector3D gravityLocal = new FrameVector3D();
      private final FrameVector3D linearVelocity = new FrameVector3D();
      private final FrameVector3D angularVelocity = new FrameVector3D();
      private final FrameVector3D northVector = new FrameVector3D();

      public void updateState()
      {
         if (parent == null)
            return;

         if (parent.isRoot())
         {
            estimatedTwist.setIncludingFrame(getBodyFrame().getTwistOfFrame());
         }
         else
         {
            getBodyFrame().getTwistRelativeToOther(parent.getBodyFrame(), twistToParent);
            estimatedTwist.setIncludingFrame(parent.getEstimatedTwist());
            estimatedTwist.changeFrame(getBodyFrame());
            estimatedTwist.add(twistToParent);
         }

         if (imuSensor != null)
         {
            estimatedTwist.changeFrame(getIMUFrame());
            linearAcceleration.setReferenceFrame(getIMUFrame());
            expectedStaticAcceleration.setMatchingFrame(gravityVector);
            expectedStaticAcceleration.negate();

            // Estimate angular velocity & orientation
            linearAcceleration.setIncludingFrame(getIMUFrame(), imuSensor.getLinearAccelerationMeasurement());
            angularVelocity.setIncludingFrame(getIMUFrame(), imuSensor.getAngularVelocityMeasurement());
            northVector.setIncludingFrame(worldFrame, Axis3D.X);
            northVector.changeFrame(getIMUFrame());
            orientationFilter.update(angularVelocity, linearAcceleration, northVector);
            IMUBasedPelvisRotationalStateUpdater.computeOrientationAtEstimateFrame(getIMUFrame(),
                                                                                   orientationFilter.getEstimatedOrientation(),
                                                                                   getBodyFrame(),
                                                                                   estimatedPose.getRotation());
            estimatedTwist.getAngularPart().set(orientationFilter.getEstimatedAngularVelocity());

            // Estimate linear velocity
            gravityLocal.setIncludingFrame(gravityVector);
            estimatedPose.getRotation().inverseTransform(gravityLocal);
            gravityLocal.setReferenceFrame(getBodyFrame());
            gravityLocal.changeFrame(getIMUFrame());
            linearAcceleration.add(imuSensor.getLinearAccelerationMeasurement(), gravityLocal);
            linearVelocityEstimate.update(estimatedTwist.getLinearPart(), linearAcceleration);
            estimatedTwist.getLinearPart().set(linearVelocityEstimate);
            estimatedTwist.changeFrame(getBodyFrame());

            // Estimate position
            linearVelocity.setIncludingFrame(estimatedTwist.getLinearPart());
            estimatedPose.getRotation().inverseTransform(linearVelocity);
            linearVelocity.setReferenceFrame(worldFrame);
            positionEstimate.update(getBodyFrame().getTransformToRoot().getTranslation(), linearVelocity);
            estimatedPose.getTranslation().set(positionEstimate);

            yoEstimatedPose.set(estimatedPose);
            yoEstimatedTwist.set(estimatedTwist);
         }
         else
         {
            if (parent.isRoot())
            {
               estimatedPose.set(getBodyFrame().getTransformToRoot());
            }
            else
            {
               getBodyFrame().getTransformToDesiredFrame(estimatedPose, parent.getBodyFrame());
               estimatedPose.preMultiply(parent.getEstimatedPose());
            }

            yoEstimatedPose.set(estimatedPose);
            yoEstimatedTwist.set(estimatedTwist);
         }

         if (MORE_VARIABLES)
         {
            yoRawPose.set(getBodyFrame().getTransformToRoot());
            yoRawTwist.set(getBodyFrame().getTwistOfFrame());
         }
      }

      public boolean isRoot()
      {
         return parent == null;
      }

      public RigidBodyTransformReadOnly getEstimatedPose()
      {
         return estimatedPose;
      }

      public TwistReadOnly getEstimatedTwist()
      {
         return estimatedTwist;
      }

      public MovingReferenceFrame getBodyFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }

      public ReferenceFrame getIMUFrame()
      {
         return imuSensor.getMeasurementFrame();
      }
   }
}
