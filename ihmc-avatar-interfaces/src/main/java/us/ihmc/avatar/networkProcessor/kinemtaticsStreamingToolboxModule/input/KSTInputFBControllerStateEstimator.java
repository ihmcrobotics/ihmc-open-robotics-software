package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

public class KSTInputFBControllerStateEstimator implements KSTInputStateEstimator
{
   private static final boolean ENABLE_VALENTINE_POWER = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final double SAFE_INPUT_PERIOD_TO_CORRECTION_FACTOR = 1.5;

   private final YoPDGains gains;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Map<RigidBodyReadOnly, SingleEndEffectorStateEstimatorBase> inputPoseEstimators = new LinkedHashMap<>();
   private final SingleEndEffectorStateEstimatorBase[] endEffectorEstimatorsArray;
   private final CenterOfMassEstimator centerOfMassEstimator;

   private final YoDouble correctionDuration = new YoDouble("correctionDuration", registry);
   private final YoDouble rawVelocityAlpha = new YoDouble("rawVelocityAlpha", registry);
   private final YoDouble maxDeltaLinearVelocity = new YoDouble("maxDeltaLinearVelocity", registry);
   private final YoDouble maxDeltaAngularVelocity = new YoDouble("maxDeltaAngularVelocity", registry);
   private final double updateDT;
   /**
    * Period at which the input is updated. This is used to clamp the correction of the estimated pose. Should preferably be filtered.
    */
   private final DoubleProvider inputPeriod;
   private final YoDouble inputVelocityDecayDuration = new YoDouble("inputVelocityDecayDuration", registry);

   public KSTInputFBControllerStateEstimator(Collection<? extends RigidBodyReadOnly> endEffectors,
                                             KinematicsStreamingToolboxParameters parameters,
                                             double updateDT,
                                             DoubleProvider inputPeriod,
                                             YoRegistry parentRegistry)
   {
      this.updateDT = updateDT;
      this.inputPeriod = inputPeriod;

      for (RigidBodyReadOnly endEffector : endEffectors)
      {
         if (ENABLE_VALENTINE_POWER)
            inputPoseEstimators.put(endEffector, new SingleEndEffectorEstimatorValentine(endEffector));
         else
            inputPoseEstimators.put(endEffector, new SingleEndEffectorEstimator(endEffector));
      }

      gains = new YoPDGains("FBC_input", registry);
      gains.createDerivativeGainUpdater(false);
      gains.setKp(500.0);
      gains.setZeta(1.0);

      endEffectorEstimatorsArray = inputPoseEstimators.values().toArray(new SingleEndEffectorStateEstimatorBase[0]);
      centerOfMassEstimator = new CenterOfMassEstimator();

      correctionDuration.set(parameters.getInputPoseCorrectionDuration());
      rawVelocityAlpha.set(parameters.getInputVelocityRawAlpha());
      maxDeltaLinearVelocity.set(2.0);
      maxDeltaAngularVelocity.set(2.0);

      inputVelocityDecayDuration.set(parameters.getInputVelocityDecayDuration());

      parentRegistry.addChild(registry);
   }

   @Override
   public void reset()
   {
      for (SingleEndEffectorStateEstimatorBase estimator : endEffectorEstimatorsArray)
      {
         estimator.reset();
      }
      centerOfMassEstimator.reset();
   }

   @Override
   public void update(double time,
                      boolean isNewInput,
                      double defaultLinearRateLimitation,
                      double defaultAngularRateLimitation,
                      KinematicsStreamingToolboxInputCommand latestInputCommand,
                      KinematicsStreamingToolboxInputCommand previousRawInputCommand)
   {
      double minCorrectionDuration = 2.0 * updateDT;
      if (inputPeriod.getValue() > updateDT)
         minCorrectionDuration = Math.max(minCorrectionDuration, SAFE_INPUT_PERIOD_TO_CORRECTION_FACTOR * inputPeriod.getValue());

      if (!Double.isFinite(correctionDuration.getValue()) || correctionDuration.getValue() < minCorrectionDuration)
         correctionDuration.set(minCorrectionDuration);

      if (isNewInput)
      {
         for (int i = 0; i < latestInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = latestInputCommand.getInput(i);

            SingleEndEffectorStateEstimatorBase inputPoseEstimator = inputPoseEstimators.get(input.getEndEffector());

            if (inputPoseEstimator != null)
            {
               inputPoseEstimator.update(time, defaultLinearRateLimitation, defaultAngularRateLimitation, latestInputCommand.getTimestamp(), input);
            }
         }

         if (latestInputCommand.hasCenterOfMassInput())
         {
            KinematicsToolboxCenterOfMassCommand input = latestInputCommand.getCenterOfMassInput();
            centerOfMassEstimator.update(time, defaultLinearRateLimitation, latestInputCommand.getTimestamp(), input);
         }
      }
      else
      {
         for (int i = 0; i < latestInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = latestInputCommand.getInput(i);

            SingleEndEffectorStateEstimatorBase inputPoseEstimator = inputPoseEstimators.get(input.getEndEffector());

            if (inputPoseEstimator != null)
               inputPoseEstimator.predict(time);
         }

         if (latestInputCommand.hasCenterOfMassInput())
         {
            centerOfMassEstimator.predict(time);
         }
      }
   }

   @Override
   public FramePose3DReadOnly getEstimatedPose(RigidBodyReadOnly endEffector)
   {
      SingleEndEffectorStateEstimatorBase inputPoseEstimator = inputPoseEstimators.get(endEffector);
      return inputPoseEstimator != null ? inputPoseEstimator.getEstimatedPose() : null;
   }

   @Override
   public SpatialVectorReadOnly getEstimatedVelocity(RigidBodyReadOnly endEffector)
   {
      SingleEndEffectorStateEstimatorBase inputPoseEstimator = inputPoseEstimators.get(endEffector);
      return inputPoseEstimator != null ? inputPoseEstimator.getEstimatedVelocity() : null;
   }

   @Override
   public FramePoint3DReadOnly getEstimatedCoMPosition()
   {
      return centerOfMassEstimator.getEstimatedPosition();
   }

   @Override
   public FrameVector3DReadOnly getEstimatedCoMVelocity()
   {
      return centerOfMassEstimator.getEstimatedVelocity();
   }

   private interface SingleEndEffectorStateEstimatorBase
   {

      void reset();

      void update(double time,
                  double defaultLinearRateLimitation,
                  double defaultAngularRateLimitation,
                  long inputTimestamp,
                  KinematicsToolboxRigidBodyCommand input);

      void predict(double time);

      FramePose3DReadOnly getEstimatedPose();

      SpatialVectorReadOnly getEstimatedVelocity();

   }

   private class SingleEndEffectorEstimatorValentine implements SingleEndEffectorStateEstimatorBase
   {
      private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private final YoFramePose3D estimatedPose;
      private final YoFixedFrameSpatialVector estimatedVelocity;
      private final YoFixedFrameSpatialVector estimatedDecayingVelocity;

      private final YoFixedFrameSpatialVector estimatedAcceleration;

      private final YoDouble lastUpdateTime;
      private final YoLong lastInputTimestamp;
      private final YoFramePose3D rawInputPose;
      private final YoFixedFrameSpatialVector rawInputVelocity;
      private final YoFixedFrameSpatialVector debugInputVelocity;
      private final YoDouble nextTimeTriggerForDecay;
      private final YoDouble inputVelocityDecayFactor;

      private FrameVector3D positionVectorError = new FrameVector3D();
      private FrameVector3D linearVelocityVectorError = new FrameVector3D();

      private Quaternion quaternionError = new Quaternion();
      private Vector3D rotationVectorErrorBodyFrame = new Vector3D();
      private Vector3D angularVelocityErrorBodyFrame = new Vector3D();

      private FrameVector3D linearFeedbackWorldFrame = new FrameVector3D();
      private FrameVector3D angularFeedbackBodyFrame = new FrameVector3D();
      private FrameVector3D angularAccelerationBodyFrame = new FrameVector3D();
      private FrameVector3D angularVelocityBodyFrame = new FrameVector3D();

      Vector3D orientationUpdate = new Vector3D();
      Quaternion deltaOrientation = new Quaternion();

      public SingleEndEffectorEstimatorValentine(RigidBodyReadOnly endEffector)
      {


         String namePrefix = endEffector.getName() + "_FBC_";
         estimatedPose = new YoFramePose3D(new YoFramePoint3D(namePrefix + "EstimatedPosition", worldFrame, registry),
                                           new YoFrameQuaternion(namePrefix + "EstimatedOrientation", worldFrame, registry));
         estimatedVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedAngularVelocity", worldFrame, registry),
                                                           new YoFrameVector3D(namePrefix + "EstimatedLinearVelocity", worldFrame, registry));
         estimatedDecayingVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedDecayingAngularVelocity", worldFrame, registry),
                                                                   new YoFrameVector3D(namePrefix + "EstimatedDecayingLinearVelocity", worldFrame, registry));

         estimatedAcceleration = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedAngularAcceleration", worldFrame, registry),
                                                               new YoFrameVector3D(namePrefix + "EstimatedLinearAcceleration", worldFrame, registry));

         debugInputVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "DebugAngularVelocity", worldFrame, registry),
                                                            new YoFrameVector3D(namePrefix + "DebugLinearVelocity", worldFrame, registry));

         lastUpdateTime = new YoDouble(namePrefix + "LastUpdateTime", registry);
         lastInputTimestamp = new YoLong(namePrefix + "LastInputTimestamp", registry);
         rawInputPose = new YoFramePose3D(new YoFramePoint3D(namePrefix + "RawInputPosition", worldFrame, registry),
                                          new YoFrameQuaternion(namePrefix + "RawInputOrientation", worldFrame, registry));
         rawInputVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "RawInputAngularVelocity", worldFrame, registry),
                                                          new YoFrameVector3D(namePrefix + "RawInputLinearVelocity", worldFrame, registry));

         nextTimeTriggerForDecay = new YoDouble(namePrefix + "NextTimeTriggerForDecay", registry);
         inputVelocityDecayFactor = new YoDouble(namePrefix + "InputVelocityDecayFactor", registry);
      }


      @Override
      public void reset()
      {
         estimatedPose.setToZero();
         estimatedVelocity.setToZero();
         estimatedDecayingVelocity.setToZero();
         estimatedAcceleration.setToZero();
         debugInputVelocity.setToZero();
         lastUpdateTime.set(Double.NaN);
         lastInputTimestamp.set(Long.MIN_VALUE);
         rawInputPose.setToZero();
         rawInputVelocity.setToZero();
         nextTimeTriggerForDecay.set(0.0);
         inputVelocityDecayFactor.set(0.0);
      }


      public void update(double time,
                         double defaultLinearRateLimitation,
                         double defaultAngularRateLimitation,
                         long inputTimestamp,
                         KinematicsToolboxRigidBodyCommand input)
      {
         //LINEAR FEEDBACK

         positionVectorError.sub(input.getControlFramePose().getPosition(), estimatedPose.getPosition());
         linearVelocityVectorError.sub(input.getDesiredVelocity().getLinearPart(), estimatedVelocity.getLinearPart());

         positionVectorError.scale(gains.getKp());
         linearVelocityVectorError.scale(gains.getKd());

         linearFeedbackWorldFrame.add(positionVectorError, linearVelocityVectorError); // This is in world-frame

         //ANGULAR FEEDBACK

         quaternionError.difference(estimatedPose.getOrientation(), input.getControlFramePose().getOrientation());
         quaternionError.getRotationVector(rotationVectorErrorBodyFrame); // expressed in body-frame estimated
         angularVelocityErrorBodyFrame.sub(input.getDesiredVelocity().getAngularPart(), estimatedVelocity.getAngularPart()); // Velocities are expressed in world frame
         estimatedPose.inverseTransform(angularVelocityErrorBodyFrame); // This is now in the body-fixed frame

         rotationVectorErrorBodyFrame.scale(gains.getKp());
         angularVelocityErrorBodyFrame.scale(gains.getKd());

         angularFeedbackBodyFrame.add(rotationVectorErrorBodyFrame, angularVelocityErrorBodyFrame); // This is in body-frame as described by the estimatedPose

         angularAccelerationBodyFrame.set(angularFeedbackBodyFrame);

         YoFrameVector3D estimatedLinearAccelerationWorldFrame = estimatedAcceleration.getLinearPart();
         YoFrameVector3D estimatedAngularAccelerationWorldFrame = estimatedAcceleration.getAngularPart();

         estimatedLinearAccelerationWorldFrame.set(linearFeedbackWorldFrame);
         estimatedPose.transform(angularAccelerationBodyFrame, estimatedAngularAccelerationWorldFrame);


         //  POSITION IN WORLDFRAME

         estimatedPose.getPosition().scaleAdd((updateDT*updateDT)/2, estimatedLinearAccelerationWorldFrame, estimatedPose.getPosition());
         estimatedPose.getPosition().scaleAdd(updateDT, estimatedVelocity.getLinearPart(), estimatedPose.getPosition());


          // LINEAR VELOCITY IN WORLDFRAME

         estimatedVelocity.getLinearPart().scaleAdd(updateDT, estimatedLinearAccelerationWorldFrame, estimatedVelocity.getLinearPart());


         //  ORIENTATION

         estimatedPose.inverseTransform(estimatedVelocity.getAngularPart(), angularVelocityBodyFrame);

         orientationUpdate.setAndScale(updateDT, angularVelocityBodyFrame);
         orientationUpdate.scaleAdd((updateDT*updateDT)/2, angularAccelerationBodyFrame, orientationUpdate);

         deltaOrientation.setRotationVector(orientationUpdate);

         estimatedPose.getOrientation().append(deltaOrientation);
         estimatedPose.transform(estimatedPose.getOrientation());

         //  ANGULAR VELOCITY

         estimatedVelocity.getAngularPart().scaleAdd(updateDT, angularAccelerationBodyFrame, estimatedVelocity.getAngularPart());
         estimatedPose.transform(estimatedVelocity.getAngularPart());

         estimatedAcceleration.set(estimatedLinearAccelerationWorldFrame,estimatedAngularAccelerationWorldFrame);




         // Notes for the integration:
         // For position: do everything in world
         // For orientation:
         // - compute acceleration in both world and body frames
         // - double integrate in body frame to update orientation.
         // - integrate acceleration to velocity in world to update velocity.
         // - at the end estimated acceleration and velocity are expressed in world


      }


      @Override
      public void predict(double time)
      {
         if (time > nextTimeTriggerForDecay.getValue())
         {
            double alpha = Math.min(1.0, inputVelocityDecayFactor.getValue() + updateDT / inputVelocityDecayDuration.getValue());
            inputVelocityDecayFactor.set(alpha);
            estimatedDecayingVelocity.getLinearPart().interpolate(estimatedVelocity.getLinearPart(), EuclidCoreTools.zeroVector3D, alpha);
            estimatedDecayingVelocity.getAngularPart().interpolate(estimatedVelocity.getAngularPart(), EuclidCoreTools.zeroVector3D, alpha);
         }

         KSTTools.integrateLinearVelocity(updateDT, estimatedPose.getPosition(), estimatedDecayingVelocity.getLinearPart(), estimatedPose.getPosition());
         KSTTools.integrateAngularVelocity(updateDT,
                                           estimatedPose.getOrientation(),
                                           estimatedDecayingVelocity.getAngularPart(),
                                           false,
                                           estimatedPose.getOrientation());
      }

      @Override
      public FramePose3DReadOnly getEstimatedPose()
      {
         return estimatedPose;
      }

      @Override
      public SpatialVectorReadOnly getEstimatedVelocity()
      {
         return estimatedDecayingVelocity;
      }
   }

   private class SingleEndEffectorEstimator implements SingleEndEffectorStateEstimatorBase
   {
      private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private final YoFramePose3D estimatedPose;
      private final YoFixedFrameSpatialVector estimatedVelocity;
      private final YoFixedFrameSpatialVector estimatedDecayingVelocity;

      private final YoFixedFrameSpatialVector correctiveVelocity;

      private final YoDouble lastUpdateTime;
      private final YoLong lastInputTimestamp;
      private final YoFramePose3D rawInputPose;
      private final YoFixedFrameSpatialVector rawInputVelocity;
      private final YoFixedFrameSpatialVector clampedInputVelocity;
      private final YoFixedFrameSpatialVector debugInputVelocity;
      private final YoDouble nextTimeTriggerForDecay;
      private final YoDouble inputVelocityDecayFactor;

      public SingleEndEffectorEstimator(RigidBodyReadOnly endEffector)
      {
         String namePrefix = endEffector.getName() + "_FBC_";
         estimatedPose = new YoFramePose3D(new YoFramePoint3D(namePrefix + "EstimatedPosition", worldFrame, registry),
                                           new YoFrameQuaternion(namePrefix + "EstimatedOrientation", worldFrame, registry));
         estimatedVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedAngularVelocity", worldFrame, registry),
                                                           new YoFrameVector3D(namePrefix + "EstimatedLinearVelocity", worldFrame, registry));
         estimatedDecayingVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedDecayingAngularVelocity", worldFrame, registry),
                                                                   new YoFrameVector3D(namePrefix + "EstimatedDecayingLinearVelocity", worldFrame, registry));

         correctiveVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "CorrectiveAngularVelocity", worldFrame, registry),
                                                            new YoFrameVector3D(namePrefix + "CorrectiveLinearVelocity", worldFrame, registry));
         debugInputVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "DebugAngularVelocity", worldFrame, registry),
                                                            new YoFrameVector3D(namePrefix + "DebugLinearVelocity", worldFrame, registry));

         lastUpdateTime = new YoDouble(namePrefix + "LastUpdateTime", registry);
         lastInputTimestamp = new YoLong(namePrefix + "LastInputTimestamp", registry);
         rawInputPose = new YoFramePose3D(new YoFramePoint3D(namePrefix + "RawInputPosition", worldFrame, registry),
                                          new YoFrameQuaternion(namePrefix + "RawInputOrientation", worldFrame, registry));
         rawInputVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "RawInputAngularVelocity", worldFrame, registry),
                                                          new YoFrameVector3D(namePrefix + "RawInputLinearVelocity", worldFrame, registry));
         clampedInputVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "ClampedInputAngularVelocity", worldFrame, registry),
                                                              new YoFrameVector3D(namePrefix + "ClampedInputLinearVelocity", worldFrame, registry));

         nextTimeTriggerForDecay = new YoDouble(namePrefix + "NextTimeTriggerForDecay", registry);
         inputVelocityDecayFactor = new YoDouble(namePrefix + "InputVelocityDecayFactor", registry);
      }

      private final Vector3D rawLinearDeltaVelocity = new Vector3D();
      private final Vector3D rawAngularDeltaVelocity = new Vector3D();

      @Override
      public void reset()
      {
         estimatedPose.setToZero();
         estimatedVelocity.setToZero();
         estimatedDecayingVelocity.setToZero();
         correctiveVelocity.setToZero();
         debugInputVelocity.setToZero();
         lastUpdateTime.set(Double.NaN);
         lastInputTimestamp.set(Long.MIN_VALUE);
         rawInputPose.setToZero();
         rawInputVelocity.setToZero();
         nextTimeTriggerForDecay.set(0.0);
         inputVelocityDecayFactor.set(0.0);
         clampedInputVelocity.setToZero();
      }

      private final Quaternion tempError = new Quaternion();

      @Override
      public void update(double time,
                         double defaultLinearRateLimitation,
                         double defaultAngularRateLimitation,
                         long inputTimestamp,
                         KinematicsToolboxRigidBodyCommand input)
      {
         if (input.getLinearRateLimitation() > 0.0)
            defaultLinearRateLimitation = input.getLinearRateLimitation();
         if (input.getAngularRateLimitation() > 0.0)
            defaultAngularRateLimitation = input.getAngularRateLimitation();

         FramePose3DReadOnly pose = input.getDesiredPose();

         if (lastUpdateTime.isNaN())
         {
            estimatedPose.set(pose);
            correctiveVelocity.setToZero();

            if (!input.getHasDesiredVelocity())
            {
               estimatedVelocity.setToZero();
               estimatedDecayingVelocity.setToZero();
            }
            else
            {
               estimatedVelocity.setMatchingFrame(input.getDesiredVelocity());
               estimatedVelocity.getLinearPart().clipToMaxNorm(defaultLinearRateLimitation);
               estimatedVelocity.getAngularPart().clipToMaxNorm(defaultAngularRateLimitation);
               estimatedDecayingVelocity.set(estimatedVelocity);
            }
         }
         else
         {
            correctiveVelocity.getLinearPart().sub(pose.getPosition(), estimatedPose.getPosition());
            tempError.difference(estimatedPose.getOrientation(), pose.getOrientation());
            tempError.normalizeAndLimitToPi();
            tempError.getRotationVector(correctiveVelocity.getAngularPart());
            correctiveVelocity.scale(1.0 / correctionDuration.getValue());
            estimatedPose.getOrientation().transform(correctiveVelocity.getAngularPart());

            if (!input.getHasDesiredVelocity())
            {
               double timeInterval = Conversions.nanosecondsToSeconds(inputTimestamp - lastInputTimestamp.getLongValue());
               KSTTools.computeSpatialVelocity(timeInterval, rawInputPose, pose, rawInputVelocity);
               rawInputPose.getOrientation().transform(rawInputVelocity.getAngularPart());
            }
            else
            {
               double timeInterval = Conversions.nanosecondsToSeconds(inputTimestamp - lastInputTimestamp.getLongValue());
               KSTTools.computeSpatialVelocity(timeInterval, rawInputPose, pose, debugInputVelocity);
               rawInputPose.getOrientation().transform(debugInputVelocity.getAngularPart());
               rawInputVelocity.setMatchingFrame(input.getDesiredVelocity());
            }

            if (rawInputVelocity.containsNaN())
            {
               rawInputVelocity.setToZero();
               clampedInputVelocity.setToZero();
            }
            else
            {
               rawLinearDeltaVelocity.sub(rawInputVelocity.getLinearPart(), clampedInputVelocity.getLinearPart());
               rawAngularDeltaVelocity.sub(rawInputVelocity.getAngularPart(), clampedInputVelocity.getAngularPart());
               rawLinearDeltaVelocity.clipToMaxNorm(maxDeltaLinearVelocity.getValue());
               rawAngularDeltaVelocity.clipToMaxNorm(maxDeltaAngularVelocity.getValue());
               clampedInputVelocity.getLinearPart().add(rawLinearDeltaVelocity);
               clampedInputVelocity.getAngularPart().add(rawAngularDeltaVelocity);
            }

            estimatedVelocity.set(clampedInputVelocity);
            estimatedVelocity.scale(rawVelocityAlpha.getValue());
            estimatedVelocity.add(correctiveVelocity);
            estimatedVelocity.getLinearPart().clipToMaxNorm(defaultLinearRateLimitation);
            estimatedVelocity.getAngularPart().clipToMaxNorm(defaultAngularRateLimitation);

            estimatedDecayingVelocity.set(estimatedVelocity);
            KSTTools.integrateLinearVelocity(updateDT, estimatedPose.getPosition(), estimatedDecayingVelocity.getLinearPart(), estimatedPose.getPosition());
            KSTTools.integrateAngularVelocity(updateDT,
                                              estimatedPose.getOrientation(),
                                              estimatedDecayingVelocity.getAngularPart(),
                                              false,
                                              estimatedPose.getOrientation());
         }

         lastUpdateTime.set(time);
         lastInputTimestamp.set(inputTimestamp);
         rawInputPose.set(pose);
         nextTimeTriggerForDecay.set(time + correctionDuration.getValue());
      }

      @Override
      public void predict(double time)
      {
         if (time > nextTimeTriggerForDecay.getValue())
         {
            double alpha = Math.min(1.0, inputVelocityDecayFactor.getValue() + updateDT / inputVelocityDecayDuration.getValue());
            inputVelocityDecayFactor.set(alpha);
            estimatedDecayingVelocity.getLinearPart().interpolate(estimatedVelocity.getLinearPart(), EuclidCoreTools.zeroVector3D, alpha);
            estimatedDecayingVelocity.getAngularPart().interpolate(estimatedVelocity.getAngularPart(), EuclidCoreTools.zeroVector3D, alpha);
         }

         KSTTools.integrateLinearVelocity(updateDT, estimatedPose.getPosition(), estimatedDecayingVelocity.getLinearPart(), estimatedPose.getPosition());
         KSTTools.integrateAngularVelocity(updateDT,
                                           estimatedPose.getOrientation(),
                                           estimatedDecayingVelocity.getAngularPart(),
                                           false,
                                           estimatedPose.getOrientation());
      }

      @Override
      public FramePose3DReadOnly getEstimatedPose()
      {
         return estimatedPose;
      }

      @Override
      public SpatialVectorReadOnly getEstimatedVelocity()
      {
         return estimatedDecayingVelocity;
      }
   }

   private class CenterOfMassEstimator
   {
      private final YoFramePoint3D estimatedPosition;
      private final YoFrameVector3D estimatedVelocity;
      private final YoFrameVector3D estimatedDecayingVelocity;

      private final YoFrameVector3D correctiveVelocity;

      private final YoDouble lastUpdateTime;
      private final YoLong lastInputTimestamp;
      private final YoFramePoint3D rawInputPosition;
      private final YoFrameVector3D rawInputVelocity;
      private final YoFrameVector3D debugInputVelocity;
      private final YoDouble nextTimeTriggerForDecay;
      private final YoDouble inputVelocityDecayFactor;

      public CenterOfMassEstimator()
      {
         String namePrefix = "CoM_FBC_";
         estimatedPosition = new YoFramePoint3D(namePrefix + "EstimatedPosition", worldFrame, registry);
         estimatedVelocity = new YoFrameVector3D(namePrefix + "EstimatedVelocity", worldFrame, registry);
         estimatedDecayingVelocity = new YoFrameVector3D(namePrefix + "EstimatedDecayingVelocity", worldFrame, registry);

         correctiveVelocity = new YoFrameVector3D(namePrefix + "CorrectiveVelocity", worldFrame, registry);
         debugInputVelocity = new YoFrameVector3D(namePrefix + "DebugInputVelocity", worldFrame, registry);

         lastUpdateTime = new YoDouble(namePrefix + "LastUpdateTime", registry);
         lastInputTimestamp = new YoLong(namePrefix + "LastInputTimestamp", registry);
         rawInputPosition = new YoFramePoint3D(namePrefix + "RawInputPosition", worldFrame, registry);
         rawInputVelocity = new YoFrameVector3D(namePrefix + "RawInputVelocity", worldFrame, registry);

         nextTimeTriggerForDecay = new YoDouble(namePrefix + "NextTimeTriggerForDecay", registry);
         inputVelocityDecayFactor = new YoDouble(namePrefix + "InputVelocityDecayFactor", registry);
      }

      public void reset()
      {
         estimatedPosition.setToZero();
         estimatedVelocity.setToZero();
         estimatedDecayingVelocity.setToZero();
         correctiveVelocity.setToZero();
         debugInputVelocity.setToZero();
         lastUpdateTime.set(Double.NaN);
         lastInputTimestamp.set(Long.MIN_VALUE);
         rawInputPosition.setToZero();
         rawInputVelocity.setToZero();
         nextTimeTriggerForDecay.set(0.0);
         inputVelocityDecayFactor.set(0.0);
      }

      public void update(double time, double defaultLinearRateLimitation, long inputTimestamp, KinematicsToolboxCenterOfMassCommand input)
      {
         if (input.getLinearRateLimitation() > 0.0)
            defaultLinearRateLimitation = input.getLinearRateLimitation();

         FramePoint3DReadOnly position = input.getDesiredPosition();

         if (lastUpdateTime.isNaN())
         {
            estimatedPosition.set(position);
            correctiveVelocity.setToZero();

            if (!input.getHasDesiredVelocity())
            {
               estimatedVelocity.setToZero();
               estimatedDecayingVelocity.setToZero();
            }
            else
            {
               estimatedVelocity.set(input.getDesiredVelocity());
               estimatedDecayingVelocity.set(estimatedVelocity);
            }
         }
         else
         {
            correctiveVelocity.sub(position, estimatedPosition);
            correctiveVelocity.scale(1.0 / correctionDuration.getValue());

            if (!input.getHasDesiredVelocity())
            {
               double timeInterval = Conversions.nanosecondsToSeconds(inputTimestamp - lastInputTimestamp.getLongValue());
               KSTTools.computeLinearVelocity(timeInterval, rawInputPosition, position, rawInputVelocity);
            }
            else
            {
               double timeInterval = Conversions.nanosecondsToSeconds(inputTimestamp - lastInputTimestamp.getLongValue());
               KSTTools.computeLinearVelocity(timeInterval, rawInputPosition, position, debugInputVelocity);
               rawInputVelocity.set(debugInputVelocity);
            }

            if (rawInputVelocity.containsNaN())
               rawInputVelocity.setToZero();

            estimatedVelocity.set(rawInputVelocity);
            estimatedVelocity.scale(rawVelocityAlpha.getValue());
            estimatedVelocity.add(correctiveVelocity);
            estimatedVelocity.clipToMaxNorm(defaultLinearRateLimitation);
            estimatedDecayingVelocity.set(estimatedVelocity);
            KSTTools.integrateLinearVelocity(updateDT, estimatedPosition, estimatedDecayingVelocity, estimatedPosition);
         }

         lastUpdateTime.set(time);
         lastInputTimestamp.set(inputTimestamp);
         rawInputPosition.set(position);
         nextTimeTriggerForDecay.set(time + correctionDuration.getValue());
         inputVelocityDecayFactor.set(0.0);
      }

      public void predict(double time)
      {
         if (time > nextTimeTriggerForDecay.getValue())
         {
            double alpha = Math.min(1.0, inputVelocityDecayFactor.getValue() + updateDT / inputVelocityDecayDuration.getValue());
            inputVelocityDecayFactor.set(alpha);
            estimatedDecayingVelocity.interpolate(estimatedVelocity, EuclidCoreTools.zeroVector3D, alpha);
         }

         KSTTools.integrateLinearVelocity(updateDT, estimatedPosition, estimatedDecayingVelocity, estimatedPosition);
      }

      public FramePoint3DReadOnly getEstimatedPosition()
      {
         return estimatedPosition;
      }

      public FrameVector3DReadOnly getEstimatedVelocity()
      {
         return estimatedDecayingVelocity;
      }
   }

   @Override
   public void initialize()
   {

   }
}
