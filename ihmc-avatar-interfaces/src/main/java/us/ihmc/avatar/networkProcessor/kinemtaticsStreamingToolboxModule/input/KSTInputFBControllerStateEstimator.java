package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
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
   public static final double SAFE_INPUT_PERIOD_TO_CORRECTION_FACTOR = 1.5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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
            inputPoseEstimators.put(endEffector, new SingleEndEffectorC1Estimator(endEffector));
         else
            inputPoseEstimators.put(endEffector, new SingleEndEffectorC0Estimator(endEffector));
      }

      endEffectorEstimatorsArray = inputPoseEstimators.values().toArray(new SingleEndEffectorStateEstimatorBase[0]);
      centerOfMassEstimator = new CenterOfMassEstimator();

      correctionDuration.set(parameters.getInputPoseCorrectionDuration());
      rawVelocityAlpha.set(parameters.getInputVelocityRawAlpha());
      inputVelocityDecayDuration.set(parameters.getInputVelocityDecayDuration());
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

   private class SingleEndEffectorC0Estimator implements SingleEndEffectorStateEstimatorBase
   {
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

      public SingleEndEffectorC0Estimator(RigidBodyReadOnly endEffector)
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
               rawInputVelocity.setToZero();
               estimatedDecayingVelocity.setToZero();
            }
            else
            {
               estimatedVelocity.setMatchingFrame(input.getDesiredVelocity());
               estimatedVelocity.getLinearPart().clipToMaxNorm(defaultLinearRateLimitation);
               estimatedVelocity.getAngularPart().clipToMaxNorm(defaultAngularRateLimitation);
               rawInputVelocity.setMatchingFrame(input.getDesiredVelocity());
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
         inputVelocityDecayFactor.set(0.0);
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

   private class SingleEndEffectorC1Estimator implements SingleEndEffectorStateEstimatorBase
   {
      private final YoFramePose3D estimatedPose;
      private final YoFixedFrameSpatialVector estimatedVelocity;
      private final YoFixedFrameSpatialVector estimatedAcceleration;
      private final YoFixedFrameSpatialVector estimatedDecayingVelocity;
      private final YoFixedFrameSpatialVector estimatedDecayingAcceleration;

      private final YoDouble maxLinearVelocity;
      private final YoDouble maxAngularVelocity;

      private final YoDouble lastUpdateTime;
      private final YoLong lastInputTimestamp;
      private final YoFramePose3D rawInputPose;
      private final YoFixedFrameSpatialVector rawInputVelocity;
      private final YoFixedFrameSpatialVector clampedInputVelocity;
      private final YoFixedFrameSpatialVector debugInputVelocity;
      private final YoDouble nextTimeTriggerForDecay;
      private final YoDouble inputVelocityDecayFactor;

      public SingleEndEffectorC1Estimator(RigidBodyReadOnly endEffector)
      {

         String namePrefix = endEffector.getName() + "_FBC_";
         estimatedPose = new YoFramePose3D(new YoFramePoint3D(namePrefix + "EstimatedPosition", worldFrame, registry),
                                           new YoFrameQuaternion(namePrefix + "EstimatedOrientation", worldFrame, registry));
         estimatedVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedAngularVelocity", worldFrame, registry),
                                                           new YoFrameVector3D(namePrefix + "EstimatedLinearVelocity", worldFrame, registry));

         estimatedAcceleration = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedAngularAcceleration", worldFrame, registry),
                                                               new YoFrameVector3D(namePrefix + "EstimatedLinearAcceleration", worldFrame, registry));

         estimatedDecayingVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedDecayingAngularVelocity", worldFrame, registry),
                                                                   new YoFrameVector3D(namePrefix + "EstimatedDecayingLinearVelocity", worldFrame, registry));
         estimatedDecayingAcceleration = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "EstimatedDecayingAngularAcceleration",
                                                                                           worldFrame,
                                                                                           registry),
                                                                       new YoFrameVector3D(namePrefix + "EstimatedDecayingLinearAcceleration",
                                                                                           worldFrame,
                                                                                           registry));

         maxLinearVelocity = new YoDouble(namePrefix + "MaxLinearVelocity", registry);
         maxAngularVelocity = new YoDouble(namePrefix + "MaxAngularVelocity", registry);

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
         estimatedAcceleration.setToZero();
         estimatedDecayingVelocity.setToZero();
         estimatedDecayingAcceleration.setToZero();
         maxLinearVelocity.set(Double.POSITIVE_INFINITY);
         maxAngularVelocity.set(Double.POSITIVE_INFINITY);
         debugInputVelocity.setToZero();
         lastUpdateTime.set(Double.NaN);
         lastInputTimestamp.set(Long.MIN_VALUE);
         rawInputPose.setToZero();
         rawInputVelocity.setToZero();
         nextTimeTriggerForDecay.set(0.0);
         inputVelocityDecayFactor.set(0.0);
         clampedInputVelocity.setToZero();
      }

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
         maxLinearVelocity.set(defaultLinearRateLimitation);
         maxAngularVelocity.set(defaultAngularRateLimitation);

         FramePose3D inputPose = input.getDesiredPose();

         if (lastUpdateTime.isNaN())
         {
            estimatedPose.set(inputPose);

            if (!input.getHasDesiredVelocity())
            {
               estimatedVelocity.setToZero();
               rawInputVelocity.setToZero();
            }
            else
            {
               estimatedVelocity.setMatchingFrame(input.getDesiredVelocity());
               estimatedVelocity.getLinearPart().clipToMaxNorm(defaultLinearRateLimitation);
               estimatedVelocity.getAngularPart().clipToMaxNorm(defaultAngularRateLimitation);
               rawInputVelocity.setMatchingFrame(input.getDesiredVelocity());
            }

            estimatedAcceleration.setToZero();
            estimatedDecayingVelocity.set(estimatedVelocity);
            estimatedDecayingAcceleration.setToZero();
         }
         else
         {
            if (!input.getHasDesiredVelocity())
            {
               double timeInterval = Conversions.nanosecondsToSeconds(inputTimestamp - lastInputTimestamp.getLongValue());
               KSTTools.computeSpatialVelocity(timeInterval, rawInputPose, inputPose, rawInputVelocity);
               rawInputPose.getOrientation().transform(rawInputVelocity.getAngularPart());
            }
            else
            {
               double timeInterval = Conversions.nanosecondsToSeconds(inputTimestamp - lastInputTimestamp.getLongValue());
               KSTTools.computeSpatialVelocity(timeInterval, rawInputPose, inputPose, debugInputVelocity);
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

            // For continuity, we start off from the decayed velocity with is the actual output of this estimator.
            estimatedVelocity.set(estimatedDecayingVelocity);
            estimateAcceleration(inputPose, clampedInputVelocity, estimatedPose, estimatedVelocity, estimatedAcceleration);
            doubleIntegrateAcceleration(0.0, defaultAngularRateLimitation, defaultLinearRateLimitation);
         }

         nextTimeTriggerForDecay.set(time + correctionDuration.getValue());
         inputVelocityDecayFactor.set(0.0);
         lastUpdateTime.set(time);
         rawInputPose.set(inputPose);
         lastInputTimestamp.set(inputTimestamp);
      }

      private final Vector3D positionVectorError = new Vector3D();
      private final Vector3D linearVelocityVectorError = new Vector3D();

      private final Quaternion quaternionError = new Quaternion();
      private final Vector3D rotationVectorErrorBodyFrame = new Vector3D();
      private final Vector3D angularVelocityErrorBodyFrame = new Vector3D();

      private final Vector3D angularAccelerationBodyFrame = new Vector3D();

      private final Vector3D rotationVectorUpdate = new Vector3D();
      private final Quaternion orientationUpdate = new Quaternion();

      /**
       * Double integrate from the acceleration to update the estimated velocity and estimated pose. 1-DoF example:<br>
       * <img
       * src="https://latex.codecogs.com/png.image?\dpi{150}\bg{beige}\left\{\begin{matrix}\ddot{x}=a\\\dot{x}^&plus;=\Delta&space;t&space;a&plus;\dot{x}^-\\x^&plus;=\frac{\Delta&space;t^2}{2}a&plus;\Delta&space;t\dot{x}^-&plus;x^-\end{matrix}\right."
       * alt="1DoF equations"><br>
       * To facilitate applying the decay, we change the integration for the position term to be function of <img
       * src="https://latex.codecogs.com/png.image?\dot{x}^+">:<br>
       * <img
       * src="https://latex.codecogs.com/png.image?\dpi{150}\bg{beige}\left\{\begin{matrix}\ddot{x}=a\\\dot{x}^&plus;=\Delta&space;t&space;a&plus;\dot{x}^-\\x^&plus;=-\frac{\Delta&space;t^2}{2}a&plus;\Delta&space;t\dot{x}^+&plus;x^-\end{matrix}\right."
       * alt="1DoF equations"><br>
       *
       * @param decayFactor        The decay factor used to decay the estimated velocity and acceleration. A value of 0.0 means no decay.
       * @param maxAngularVelocity The maximum angular velocity allowed.
       * @param maxLinearVelocity  The maximum linear velocity allowed.
       */
      private void doubleIntegrateAcceleration(double decayFactor, double maxAngularVelocity, double maxLinearVelocity)
      {
         // Applying decay on the acceleration:
         estimatedDecayingAcceleration.getLinearPart().interpolate(estimatedAcceleration.getLinearPart(), EuclidCoreTools.zeroVector3D, decayFactor);
         estimatedDecayingAcceleration.getAngularPart().interpolate(estimatedAcceleration.getAngularPart(), EuclidCoreTools.zeroVector3D, decayFactor);

         YoFrameVector3D linearAccelerationWorldFrame = estimatedDecayingAcceleration.getLinearPart();
         YoFrameVector3D angularAccelerationWorldFrame = estimatedDecayingAcceleration.getAngularPart();
         YoFrameVector3D linearVelocityWorldFrame = estimatedVelocity.getLinearPart();
         YoFrameVector3D angularVelocityWorldFrame = estimatedVelocity.getAngularPart();
         clampAcceleration(linearAccelerationWorldFrame, updateDT, linearVelocityWorldFrame, maxLinearVelocity);
         clampAcceleration(angularAccelerationWorldFrame, updateDT, angularVelocityWorldFrame, maxAngularVelocity);
         YoFramePoint3D position = estimatedPose.getPosition();
         YoFrameQuaternion orientation = estimatedPose.getOrientation();

         // Linear Velocity
         linearVelocityWorldFrame.scaleAdd(updateDT, linearAccelerationWorldFrame, linearVelocityWorldFrame);
         estimatedDecayingVelocity.getLinearPart().interpolate(estimatedVelocity.getLinearPart(), EuclidCoreTools.zeroVector3D, decayFactor);
         linearVelocityWorldFrame = estimatedDecayingVelocity.getLinearPart();
         // Position
         position.scaleAdd(-0.5 * updateDT * updateDT, linearAccelerationWorldFrame, position);
         position.scaleAdd(updateDT, linearVelocityWorldFrame, position);

         // Angular Velocity
         angularVelocityWorldFrame.scaleAdd(updateDT, angularAccelerationWorldFrame, angularVelocityWorldFrame);
         estimatedDecayingVelocity.getAngularPart().interpolate(estimatedVelocity.getAngularPart(), EuclidCoreTools.zeroVector3D, decayFactor);
         angularVelocityWorldFrame = estimatedVelocity.getAngularPart();

         // Orientation
         rotationVectorUpdate.setAndScale(-0.5 * updateDT * updateDT, angularAccelerationWorldFrame);
         rotationVectorUpdate.scaleAdd(updateDT, angularVelocityWorldFrame, rotationVectorUpdate);
         orientationUpdate.setRotationVector(rotationVectorUpdate);
         // Using prepend instead of append because the rotation update is in world frame
         // If the update was in body frame, we would use append.
         orientation.prepend(orientationUpdate);
      }

      /**
       * Clamps the acceleration to ensure that the velocity does not exceed the maximum velocity after integration.
       * This is done by computing the velocity at the next time step.
       *
       * @param acceleration The acceleration to clamp. Modified.
       * @param dt           The time step.
       * @param velocity     The current velocity.
       * @param maxVelocity  The maximum velocity.
       */
      private static void clampAcceleration(Vector3DBasics acceleration, double dt, Vector3DReadOnly velocity, double maxVelocity)
      {
         double vx = velocity.getX() + acceleration.getX() * dt;
         double vy = velocity.getY() + acceleration.getY() * dt;
         double vz = velocity.getZ() + acceleration.getZ() * dt;
         double norm = EuclidCoreTools.norm(vx, vy, vz);

         if (norm > maxVelocity)
         {
            acceleration.set(vx, vy, vz);
            acceleration.scale(maxVelocity / norm);
            acceleration.sub(velocity);
            acceleration.scale(1.0 / dt);
         }
      }

      /**
       * Between updates, the acceleration remains constant. In the 1-DoF case we have the following equations:<br>
       * <img
       * src="https://latex.codecogs.com/png.image?\dpi{150}\bg{beige}\left\{\begin{matrix}\ddot{x}=a\\\dot{x}^&plus;=\Delta&space;t&space;a&plus;\dot{x}^-\\x^&plus;=\frac{\Delta&space;t^2}{2}a&plus;\Delta&space;t\dot{x}^-&plus;x^-\end{matrix}\right."
       * alt="1DoF equations"><br>
       * The idea is that when receiving a new input, we want to find the acceleration that will correct the estimated pose to either:<br>
       * <img
       * src="https://latex.codecogs.com/png.image?\dpi{150}\bg{beige}x^+=\left\{\begin{matrix}x^{in}\\x^{in}&plus;T^{corr}\dot{x}^{in}\end{matrix}\right."
       * alt="Desired position"><br>
       * where <tt>T<sup>corr</sup></tt> is the time it takes to correct the pose.
       * Computing the acceleration to target the first solution will add delay since we will only reach the new input pose only after the correction duration.
       * The second solution will be more reactive and insert less delay, but it will tend to overshoot the target.
       * The solution is to blend the two solutions using a user defined alpha factor ({@link #rawVelocityAlpha}).
       * We rewrite the target using &alpha;:<br>
       * <img
       * src="https://latex.codecogs.com/png.image?\dpi{150}\bg{beige}x^+=x^{in}&plus;\alpha&space;T^{corr}\dot{x}^{in}"
       * alt="Desired position with alpha"><br>
       * Let's rewrite the position equation using the new target:<br>
       * <img
       * src="https://latex.codecogs.com/png.image?\dpi{150}\bg{beige}x^{in}&plus;\alpha&space;T^{corr}\dot{x}^{in}=\frac{{T^{corr}}^2}{2}a&plus;T^{corr}\dot{x}^-&plus;x^-"
       * alt="Position equation with alpha"><br>
       * From this equation, we can isolate the acceleration:<br>
       * <img
       * src="https://latex.codecogs.com/png.image?\dpi{150}\bg{beige}a=\frac{2}{T^{corr^2}}(x^{in}-x^-)&plus;\frac{2}{T^{corr}}(\alpha&space;\dot{x}^{in}-\dot{x}^-)"
       * alt="Acceleration equation"><br>
       *
       * @param inputPose     The desired pose <img src="https://latex.codecogs.com/png.image?x^{in}"> in the equations. Not modified.
       * @param inputVelocity The desired velocity <img src="https://latex.codecogs.com/png.image?\dot{x}^{in}"> in the equations. Not modified.
       */
      private void estimateAcceleration(FramePose3DReadOnly inputPose,
                                        SpatialVectorReadOnly inputVelocity,
                                        FramePose3DReadOnly previousPose,
                                        SpatialVectorReadOnly previousVelocity,
                                        FixedFrameSpatialVectorBasics outputAcceleration)
      {
         FixedFrameVector3DBasics estimatedLinearAccelerationWorldFrame = outputAcceleration.getLinearPart();
         FixedFrameVector3DBasics estimatedAngularAccelerationWorldFrame = outputAcceleration.getAngularPart();

         double kp = 2.0 / (correctionDuration.getValue() * correctionDuration.getValue());
         double kd = 2.0 / correctionDuration.getValue();

         // LINEAR FEEDBACK
         positionVectorError.sub(inputPose.getPosition(), previousPose.getPosition());
         positionVectorError.scale(kp);
         linearVelocityVectorError.scaleSub(rawVelocityAlpha.getValue(), inputVelocity.getLinearPart(), previousVelocity.getLinearPart());
         linearVelocityVectorError.scale(kd);
         estimatedLinearAccelerationWorldFrame.add(positionVectorError, linearVelocityVectorError);

         // ANGULAR FEEDBACK
         quaternionError.difference(previousPose.getOrientation(), inputPose.getOrientation());
         quaternionError.getRotationVector(rotationVectorErrorBodyFrame); // expressed in body-frame estimated
         angularVelocityErrorBodyFrame.scaleSub(rawVelocityAlpha.getValue(),
                                                inputVelocity.getAngularPart(),
                                                previousVelocity.getAngularPart()); // Velocities are expressed in world frame
         previousPose.inverseTransform(angularVelocityErrorBodyFrame); // This is now in the body-fixed frame

         rotationVectorErrorBodyFrame.scale(kp);
         angularVelocityErrorBodyFrame.scale(kd);

         // This is in body-frame as described by the estimatedPose
         angularAccelerationBodyFrame.add(rotationVectorErrorBodyFrame, angularVelocityErrorBodyFrame);
         previousPose.transform(angularAccelerationBodyFrame, estimatedAngularAccelerationWorldFrame);
      }

      /**
       * Predict the estimated pose and velocity at the next time step.
       * The prediction is performed by integrating the acceleration over the time step and is used when no new input has been received.
       *
       * @param time the duration to integrate over.
       */
      @Override
      public void predict(double time)
      {
         double decayFactor = 0.0;

         if (time > nextTimeTriggerForDecay.getValue())
         {
            decayFactor = Math.min(1.0, inputVelocityDecayFactor.getValue() + updateDT / inputVelocityDecayDuration.getValue());
            inputVelocityDecayFactor.set(decayFactor);
         }

         doubleIntegrateAcceleration(decayFactor, maxAngularVelocity.getValue(), maxLinearVelocity.getValue());
         lastUpdateTime.set(time);
      }

      @Override
      public FramePose3DReadOnly getEstimatedPose()
      {
         return estimatedPose;
      }

      @Override
      public SpatialVectorReadOnly getEstimatedVelocity()
      {
         return estimatedVelocity;
      }
   }

   // TODO Luigi: Pattern match the SingleEndEffectorStateEstimatorBase structure to update the CoM estimator.
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
}