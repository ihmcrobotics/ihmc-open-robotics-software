package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output.KSTOutputDataReadOnly;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output.YoKinematicsToolboxOutputStatus;
import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBQuaternion3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBVector6D;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
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
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Map<RigidBodyReadOnly, SingleEndEffectorEstimator> inputPoseEstimators = new LinkedHashMap<>();
   private final SingleEndEffectorEstimator[] endEffectorEstimatorsArray;
   private final CenterOfMassEstimator centerOfMassEstimator;

   /**
    * Period at which the input is updated. This is used to clamp the correction of the estimated pose. Should preferably be filtered.
    */
   private final DoubleProvider inputPeriod;
   private final YoDouble inputVelocityDecayDuration = new YoDouble("inputVelocityDecayDuration", registry);

   private final Quaternion diff = new Quaternion();
   private final SpatialVector feedbackPosition = new SpatialVector();
   private final SpatialVector feedbackVelocity = new SpatialVector();
   private final Twist currentTwist = new Twist();

   private final FrameVector3D linearFeedback = new FrameVector3D();
   private final FrameVector3D angularFeedback = new FrameVector3D();

   private final FixedFrameVector3DBasics currentRotationVector;
   private final FramePose2DReadOnly desiredPose;

   protected final YoPDGains gains;
   private final YoPID3DGains positionGains;
   private final YoPID3DGains orientationGains;

   private ReferenceFrame controlFrame;
   private FBVector6D errorVector;
   private int currentCommandId;
   private FBQuaternion3D errorOrientation;
   private ReferenceFrame linearGainsFrame;
   private ReferenceFrame angularGainsFrame;
   private final Matrix3D tempGainMatrix = new Matrix3D();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final YoDouble correctionDuration = new YoDouble("correctionDuration", registry);
   private final YoDouble rawVelocityAlpha = new YoDouble("rawVelocityAlpha", registry);
   private final double updateDT;

   @Override
   public void initialize()
   {
      gains.createDerivativeGainUpdater(false);
      gains.setKp(500.0);
      gains.setZeta(1.0);

   }

   public KSTInputFBControllerStateEstimator(Collection<? extends RigidBodyReadOnly> endEffectors,
                                             KinematicsStreamingToolboxParameters parameters,
                                             double updateDT,
                                             DoubleProvider inputPeriod,
                                             YoRegistry parentRegistry,
                                             YoPID3DGains orientationGains)
   {

      this.currentRotationVector = null;
      this.desiredPose = null;
      this.gains = null;
      this.positionGains = null;
      this.orientationGains = orientationGains;
      this.updateDT = updateDT;
      this.inputPeriod = inputPeriod;

      for (RigidBodyReadOnly endEffector : endEffectors)
      {
         inputPoseEstimators.put(endEffector, new SingleEndEffectorEstimator(endEffector));
      }

      endEffectorEstimatorsArray = inputPoseEstimators.values().toArray(new SingleEndEffectorEstimator[0]);
      centerOfMassEstimator = new CenterOfMassEstimator();

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

      if (isNewInput)

      {

         for (int i = 0; i < latestInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand inputDesired = latestInputCommand.getInput(i);
            KinematicsToolboxRigidBodyCommand inputCurrent = previousRawInputCommand.getInput(i);

            ReferenceFrame trajectoryFrame = inputDesired.getDesiredPose().getReferenceFrame();

            inputCurrent.getDesiredPose().setToZero(controlFrame);
            inputCurrent.getDesiredPose().changeFrame(trajectoryFrame);
            inputCurrent.getDesiredPose().getOrientation().getRotationVector(currentRotationVector);

            inputDesired.getDesiredPose().setIncludingFrame(desiredPose);
            inputDesired.getDesiredPose().changeFrame(controlFrame);

            //LINEAR PART

            FrameVector3D q_err_linear = new FrameVector3D();
            q_err_linear.sub(inputDesired.getControlFramePose().getPosition(), inputCurrent.getControlFramePose().getPosition());

            FrameVector3D qd_err_linear = new FrameVector3D();
            FrameVector3D linearVelocityDesired = new FrameVector3D(inputDesired.getDesiredVelocity().getLinearPart());
            FrameVector3D linearVelocityCurrent = new FrameVector3D(inputCurrent.getDesiredVelocity().getLinearPart());

            qd_err_linear.sub(linearVelocityDesired, linearVelocityCurrent);

            q_err_linear.scale(gains.getKp());
            qd_err_linear.scale(gains.getKd());

            linearFeedback.add(q_err_linear, qd_err_linear);

            //ANGULAR PART
            FrameVector3D q_err_angular = new FrameVector3D();
            q_err_angular.sub(inputDesired.getControlFramePose().getPosition(), inputCurrent.getControlFramePose().getPosition());
            q_err_angular.scale(gains.getKp());

            Quaternion angularVelocityDesired = new Quaternion(inputDesired.getDesiredVelocity().getAngularPart());
            Quaternion angularVelocityCurrent = new Quaternion(inputCurrent.getDesiredVelocity().getAngularPart());

            Quaternion qd_err_angular = new Quaternion();
            qd_err_angular.difference(angularVelocityCurrent, angularVelocityDesired);

            //log for quaternion

            double x = qd_err_angular.getX();
            double y = qd_err_angular.getY();
            double z = qd_err_angular.getZ();
            double w = qd_err_angular.getS();
            double norm = qd_err_angular.norm();
            double vectorNorm = Math.sqrt(x * x + y * y + z * z);

            double theta = Math.acos(w / norm);
            double scale = theta / vectorNorm;

            FrameVector3D v_err = new FrameVector3D(trajectoryFrame, x * scale, y * scale, z * scale);
            v_err.scale(gains.getKd());

            angularFeedback.add(q_err_angular, v_err);

            inputDesired.getDesiredPose().getOrientation().normalizeAndLimitToPi();
            linearFeedback.setIncludingFrame(inputDesired.getDesiredPose().getPosition());
            inputDesired.getDesiredPose().getRotationVector(angularFeedback);

            selectionMatrix.applyLinearSelection(linearFeedback);
            selectionMatrix.applyAngularSelection(angularFeedback);

            linearFeedback.clipToMaxNorm(positionGains.getMaximumProportionalError());
            angularFeedback.clipToMaxNorm(orientationGains.getMaximumProportionalError());

            errorVector.setIncludingFrame(angularFeedback, linearFeedback);
            errorVector.changeFrame(controlFrame);
            errorVector.setCommandId(currentCommandId);
            errorOrientation.setRotationVectorIncludingFrame(errorVector.getAngularPart());
            errorOrientation.setCommandId(currentCommandId);

            if (linearGainsFrame != null)
               linearFeedback.changeFrame(linearGainsFrame);
            else
               linearFeedback.changeFrame(controlFrame);

            if (angularGainsFrame != null)
               angularFeedback.changeFrame(angularGainsFrame);
            else
               angularFeedback.changeFrame(controlFrame);

            positionGains.getProportionalGainMatrix(tempGainMatrix);
            tempGainMatrix.transform(linearFeedback);

            orientationGains.getProportionalGainMatrix(tempGainMatrix);
            tempGainMatrix.transform(angularFeedback);

            linearFeedback.changeFrame(controlFrame);
            angularFeedback.changeFrame(controlFrame);

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

   private class SingleEndEffectorEstimator
   {
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
         estimatedDecayingVelocity.setToZero();
         correctiveVelocity.setToZero();
         debugInputVelocity.setToZero();
         lastUpdateTime.set(Double.NaN);
         lastInputTimestamp.set(Long.MIN_VALUE);
         rawInputPose.setToZero();
         rawInputVelocity.setToZero();
         nextTimeTriggerForDecay.set(0.0);
         inputVelocityDecayFactor.set(0.0);
      }

      private final Quaternion tempError = new Quaternion();

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
               rawInputVelocity.setToZero();

            estimatedVelocity.set(rawInputVelocity);
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

      public FramePose3DReadOnly getEstimatedPose()
      {
         return estimatedPose;
      }

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
