package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
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
   public static final double SAFE_INPUT_PERIOD_TO_CORRECTION_FACTOR = 1.5;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Map<RigidBodyReadOnly, SingleEndEffectorEstimator> inputPoseEstimators = new LinkedHashMap<>();
   private final SingleEndEffectorEstimator[] endEffectorEstimatorsArray;

   private final YoDouble correctionDuration = new YoDouble("correctionDuration", registry);
   private final YoDouble rawVelocityAlpha = new YoDouble("rawVelocityAlpha", registry);
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
         inputPoseEstimators.put(endEffector, new SingleEndEffectorEstimator(endEffector));
      }

      endEffectorEstimatorsArray = inputPoseEstimators.values().toArray(new SingleEndEffectorEstimator[0]);

      correctionDuration.set(parameters.getInputPoseCorrectionDuration());
      rawVelocityAlpha.set(parameters.getInputVelocityRawAlpha());
      inputVelocityDecayDuration.set(parameters.getInputVelocityDecayDuration());

      parentRegistry.addChild(registry);
   }

   @Override
   public void reset()
   {
      for (SingleEndEffectorEstimator estimator : endEffectorEstimatorsArray)
      {
         estimator.reset();
      }
   }

   @Override
   public void update(double time,
                      boolean isNewInput,
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

            SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(input.getEndEffector());

            if (inputPoseEstimator != null)
               inputPoseEstimator.update(time, latestInputCommand.getTimestamp(), input);
         }
      }
      else
      {
         for (int i = 0; i < latestInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = latestInputCommand.getInput(i);

            SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(input.getEndEffector());

            if (inputPoseEstimator != null)
               inputPoseEstimator.predict(time);
         }
      }
   }

   @Override
   public FramePose3DReadOnly getEstimatedPose(RigidBodyReadOnly endEffector)
   {
      SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(endEffector);
      return inputPoseEstimator != null ? inputPoseEstimator.getEstimatedPose() : null;
   }

   @Override
   public SpatialVectorReadOnly getEstimatedVelocity(RigidBodyReadOnly endEffector)
   {
      SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(endEffector);
      return inputPoseEstimator != null ? inputPoseEstimator.getEstimatedVelocity() : null;
   }

   private class SingleEndEffectorEstimator
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

         nextTimeTriggerForDecay = new YoDouble(namePrefix + "NextTimeTriggerForDecay", registry);
         inputVelocityDecayFactor = new YoDouble(namePrefix + "InputVelocityDecayFactor", registry);
      }

      public void reset()
      {
         estimatedPose.setToZero();
         estimatedVelocity.setToZero();
         correctiveVelocity.setToZero();
         lastUpdateTime.set(Double.NaN);
         lastInputTimestamp.set(Long.MIN_VALUE);
         rawInputPose.setToZero();
         rawInputVelocity.setToZero();
      }

      private final Quaternion tempError = new Quaternion();

      public void update(double time, long inputTimestamp, KinematicsToolboxRigidBodyCommand input)
      {
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
            rawInputPose.getOrientation().transform(correctiveVelocity.getAngularPart()); // TODO Double check if this transform is actually needed

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

            estimatedVelocity.set(rawInputVelocity);
            estimatedVelocity.scale(rawVelocityAlpha.getValue());
            estimatedVelocity.add(correctiveVelocity);
            estimatedDecayingVelocity.set(estimatedVelocity);
            KSTTools.integrateLinearVelocity(updateDT, estimatedPose.getPosition(), estimatedDecayingVelocity.getLinearPart(), estimatedPose.getPosition());
            KSTTools.integrateAngularVelocity(updateDT, estimatedPose.getOrientation(), estimatedDecayingVelocity.getAngularPart(), false, estimatedPose.getOrientation());
         }

         lastUpdateTime.set(time);
         lastInputTimestamp.set(inputTimestamp);
         rawInputPose.set(pose);
         nextTimeTriggerForDecay.set(time + correctionDuration.getValue());
      }

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
         KSTTools.integrateAngularVelocity(updateDT, estimatedPose.getOrientation(), estimatedDecayingVelocity.getAngularPart(), false, estimatedPose.getOrientation());
      }

      public FramePose3DReadOnly getEstimatedPose()
      {
         return estimatedPose;
      }

      public SpatialVectorReadOnly getEstimatedVelocity()
      {
         return estimatedDecayingVelocity;
      }
   }
}
