package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.math.filters.RateLimitedYoSpatialVector;
import us.ihmc.robotics.math.frames.YoSpatialVector;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.POSITION;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.ROTATION_VECTOR;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.*;

public class SpatialFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoBoolean isEnabled;

   private final YoFramePose3D yoDesiredPose;
   private final YoFramePose3D yoCurrentPose;

   private final YoSpatialVector yoErrorVector;
   private final YoFrameQuaternion yoErrorOrientation;

   private final YoFrameVector3D yoErrorPositionIntegrated;
   private final YoFrameQuaternion yoErrorOrientationCumulated;
   private final YoFrameVector3D yoErrorRotationVectorIntegrated;

   private final YoSpatialVector yoDesiredVelocity;
   private final YoSpatialVector yoCurrentVelocity;
   private final YoSpatialVector yoErrorVelocity;
   private final YoSpatialVector yoFeedForwardVelocity;
   private final YoSpatialVector yoFeedbackVelocity;
   private final RateLimitedYoSpatialVector rateLimitedFeedbackVelocity;

   private final YoSpatialVector yoDesiredAcceleration;
   private final YoSpatialVector yoFeedForwardAcceleration;
   private final YoSpatialVector yoFeedbackAcceleration;
   private final RateLimitedYoSpatialVector rateLimitedFeedbackAcceleration;
   private final YoSpatialVector yoAchievedAcceleration;

   private final YoSpatialVector yoDesiredWrench;
   private final YoSpatialVector yoFeedForwardWrench;
   private final YoSpatialVector yoFeedbackWrench;
   private final RateLimitedYoSpatialVector rateLimitedFeedbackWrench;

   private final YoFrameVector3D yoDesiredRotationVector;
   private final YoFrameVector3D yoCurrentRotationVector;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FramePose3D currentPose = new FramePose3D();
   private final FramePose3D desiredPose = new FramePose3D();

   private final FrameQuaternion errorOrientationCumulated = new FrameQuaternion();

   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D currentLinearVelocity = new FrameVector3D();
   private final FrameVector3D currentAngularVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAngularVelocity = new FrameVector3D();

   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();
   private final FrameVector3D desiredAngularAcceleration = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAction = new FrameVector3D();
   private final FrameVector3D feedForwardAngularAction = new FrameVector3D();
   private final FrameVector3D biasLinearAcceleration = new FrameVector3D();
   private final FrameVector3D achievedAngularAcceleration = new FrameVector3D();
   private final FrameVector3D achievedLinearAcceleration = new FrameVector3D();

   private final FrameVector3D desiredLinearForce = new FrameVector3D();
   private final FrameVector3D desiredAngularTorque = new FrameVector3D();

   private final Twist currentTwist = new Twist();
   private final SpatialAccelerationVector endEffectorAchievedAcceleration = new SpatialAccelerationVector();

   private final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();
   private final VirtualWrenchCommand virtualModelControlOutput = new VirtualWrenchCommand();
   private final MomentumRateCommand virtualModelControlRootOutput = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final YoPIDSE3Gains gains;
   private final YoPID3DGains positionGains;
   private final YoPID3DGains orientationGains;
   private final Matrix3D tempGainMatrix = new Matrix3D();

   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final RigidBody rootBody;
   private RigidBody base;
   private ReferenceFrame controlBaseFrame;
   private ReferenceFrame angularGainsFrame;
   private ReferenceFrame linearGainsFrame;

   private final RigidBody endEffector;
   private final YoSE3OffsetFrame controlFrame;

   private final double dt;

   public SpatialFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
                                    YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;

      if (toolbox.getRootJoint() != null)
         rootBody = toolbox.getRootJoint().getSuccessor();
      else
         rootBody = null;

      spatialAccelerationCalculator = toolbox.getSpatialAccelerationCalculator();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "SpatialFBController");
      dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getSE3PIDGains(endEffector);
      positionGains = gains.getPositionGains();
      orientationGains = gains.getOrientationGains();
      YoDouble maximumLinearRate = positionGains.getYoMaximumFeedbackRate();
      YoDouble maximumAngularRate = orientationGains.getYoMaximumFeedbackRate();

      controlFrame = feedbackControllerToolbox.getControlFrame(endEffector);

      isEnabled = new YoBoolean(endEffectorName + "isSpatialFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredPose = feedbackControllerToolbox.getPose(endEffector, DESIRED, isEnabled);
      yoCurrentPose = feedbackControllerToolbox.getPose(endEffector, CURRENT, isEnabled);
      YoFrameVector3D errorPosition = feedbackControllerToolbox.getDataVector(endEffector, ERROR, POSITION, isEnabled);
      YoFrameVector3D errorRotationVector = feedbackControllerToolbox.getDataVector(endEffector, ERROR, ROTATION_VECTOR, isEnabled);
      yoErrorVector = new YoSpatialVector(errorPosition, errorRotationVector);
      yoErrorOrientation = feedbackControllerToolbox.getOrientation(endEffector, ERROR, isEnabled);
      yoErrorPositionIntegrated = feedbackControllerToolbox.getDataVector(endEffector, ERROR_INTEGRATED, POSITION, isEnabled);
      yoErrorOrientationCumulated = feedbackControllerToolbox.getOrientation(endEffector, ERROR_CUMULATED, isEnabled);
      yoErrorRotationVectorIntegrated = feedbackControllerToolbox.getDataVector(endEffector, ERROR_INTEGRATED, ROTATION_VECTOR, isEnabled);

      yoDesiredRotationVector = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ROTATION_VECTOR, isEnabled);
      yoCurrentRotationVector = feedbackControllerToolbox.getDataVector(endEffector, CURRENT, ROTATION_VECTOR, isEnabled);

      yoDesiredVelocity = feedbackControllerToolbox.getVelocity(endEffector, DESIRED, isEnabled);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {

         yoCurrentVelocity = feedbackControllerToolbox.getVelocity(endEffector, CURRENT, isEnabled);
         yoErrorVelocity = feedbackControllerToolbox.getVelocity(endEffector, ERROR, isEnabled);

         if (toolbox.isEnableInverseDynamicsModule())
         {
            yoDesiredAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, DESIRED, isEnabled);
            yoFeedForwardAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, FEEDFORWARD, isEnabled);
            yoFeedbackAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, FEEDBACK, isEnabled);
            rateLimitedFeedbackAcceleration = feedbackControllerToolbox
                  .getRateLimitedAcceleration(endEffector, FEEDBACK, dt, maximumLinearRate, maximumAngularRate, isEnabled);
            yoAchievedAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, ACHIEVED, isEnabled);
         }
         else
         {
            yoDesiredAcceleration = null;
            yoFeedForwardAcceleration = null;
            yoFeedbackAcceleration = null;
            rateLimitedFeedbackAcceleration = null;
            yoAchievedAcceleration = null;
         }

         if (toolbox.isEnableVirtualModelControlModule())
         {
            yoDesiredWrench = feedbackControllerToolbox.getWrench(endEffector, DESIRED, isEnabled);
            yoFeedForwardWrench = feedbackControllerToolbox.getWrench(endEffector, FEEDFORWARD, isEnabled);
            yoFeedbackWrench = feedbackControllerToolbox.getWrench(endEffector, FEEDBACK, isEnabled);
            rateLimitedFeedbackWrench = feedbackControllerToolbox
                  .getRateLimitedWrench(endEffector, FEEDBACK, dt, maximumLinearRate, maximumAngularRate, isEnabled);
         }
         else
         {
            yoDesiredWrench = null;
            yoFeedForwardWrench = null;
            yoFeedbackWrench = null;
            rateLimitedFeedbackWrench = null;
         }
      }
      else
      {
         yoCurrentVelocity = null;
         yoErrorVelocity = null;

         yoDesiredAcceleration = null;
         yoFeedForwardAcceleration = null;
         yoFeedbackAcceleration = null;
         rateLimitedFeedbackAcceleration = null;
         yoAchievedAcceleration = null;

         yoDesiredWrench = null;
         yoFeedForwardWrench = null;
         yoFeedbackWrench = null;
         rateLimitedFeedbackWrench = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackVelocity = feedbackControllerToolbox.getVelocity(endEffector, FEEDBACK, isEnabled);
         yoFeedForwardVelocity = feedbackControllerToolbox.getVelocity(endEffector, FEEDFORWARD, isEnabled);
         rateLimitedFeedbackVelocity = feedbackControllerToolbox
               .getRateLimitedVelocity(endEffector, FEEDBACK, dt, maximumLinearRate, maximumAngularRate, isEnabled);
      }
      else
      {
         yoFeedbackVelocity = null;
         yoFeedForwardVelocity = null;
         rateLimitedFeedbackVelocity = null;
      }

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(SpatialFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();
      controlBaseFrame = command.getControlBaseFrame();
      inverseDynamicsOutput.set(command.getSpatialAccelerationCommand());
      inverseKinematicsOutput.setProperties(command.getSpatialAccelerationCommand());
      virtualModelControlOutput.setProperties(command.getSpatialAccelerationCommand());

      gains.set(command.getGains());
      command.getSpatialAccelerationCommand().getSelectionMatrix(selectionMatrix);
      angularGainsFrame = command.getAngularGainsFrame();
      linearGainsFrame = command.getLinearGainsFrame();

      command.getControlFramePoseIncludingFrame(desiredPosition, desiredOrientation);
      controlFrame.setOffsetToParent(desiredPosition, desiredOrientation);

      command.getIncludingFrame(desiredPosition, desiredLinearVelocity);
      command.getIncludingFrame(desiredOrientation, desiredAngularVelocity);
      command.getFeedForwardActionIncludingFrame(feedForwardAngularAction, feedForwardLinearAction);

      yoDesiredPose.setMatchingFrame(desiredPosition, desiredOrientation);
      yoDesiredPose.getOrientation().getRotationVector(yoDesiredRotationVector);
      yoDesiredVelocity.setAndMatchFrame(desiredLinearVelocity, desiredAngularVelocity);

      if (yoFeedForwardVelocity != null)
         yoFeedForwardVelocity.setAndMatchFrame(feedForwardLinearAction, feedForwardAngularAction);

      if (yoFeedForwardAcceleration != null)
         yoFeedForwardAcceleration.setAndMatchFrame(feedForwardLinearAction, feedForwardAngularAction);

      if (yoFeedForwardWrench != null)
         yoFeedForwardWrench.setAndMatchFrame(feedForwardLinearAction, feedForwardAngularAction);
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   {
      if (rateLimitedFeedbackAcceleration != null)
         rateLimitedFeedbackAcceleration.reset();
      if (rateLimitedFeedbackVelocity != null)
         rateLimitedFeedbackVelocity.reset();
   }

   private final FrameVector3D linearProportionalFeedback = new FrameVector3D();
   private final FrameVector3D linearDerivativeFeedback = new FrameVector3D();
   private final FrameVector3D linearIntegralFeedback = new FrameVector3D();

   private final FrameVector3D angularProportionalFeedback = new FrameVector3D();
   private final FrameVector3D angularDerivativeFeedback = new FrameVector3D();
   private final FrameVector3D angularIntegralFeedback = new FrameVector3D();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      computeProportionalTerm(linearProportionalFeedback, angularProportionalFeedback);
      computeDerivativeTerm(linearDerivativeFeedback, angularDerivativeFeedback);
      computeIntegralTerm(linearIntegralFeedback, angularIntegralFeedback);
      yoFeedForwardAcceleration.getIncludingFrame(feedForwardLinearAction, feedForwardAngularAction);
      feedForwardLinearAction.changeFrame(controlFrame);
      feedForwardAngularAction.changeFrame(controlFrame);

      desiredLinearAcceleration.setIncludingFrame(linearProportionalFeedback);
      desiredLinearAcceleration.add(linearDerivativeFeedback);
      desiredLinearAcceleration.add(linearIntegralFeedback);
      desiredLinearAcceleration.clipToMaxLength(positionGains.getMaximumFeedback());

      desiredAngularAcceleration.setIncludingFrame(angularProportionalFeedback);
      desiredAngularAcceleration.add(angularDerivativeFeedback);
      desiredAngularAcceleration.add(angularIntegralFeedback);
      desiredAngularAcceleration.clipToMaxLength(orientationGains.getMaximumFeedback());

      yoFeedbackAcceleration.setAndMatchFrame(desiredLinearAcceleration, desiredAngularAcceleration);
      rateLimitedFeedbackAcceleration.update();
      rateLimitedFeedbackAcceleration.getIncludingFrame(desiredLinearAcceleration, desiredAngularAcceleration);

      desiredLinearAcceleration.changeFrame(controlFrame);
      desiredLinearAcceleration.add(feedForwardLinearAction);

      desiredAngularAcceleration.changeFrame(controlFrame);
      desiredAngularAcceleration.add(feedForwardAngularAction);

      yoDesiredAcceleration.setAndMatchFrame(desiredLinearAcceleration, desiredAngularAcceleration);

      addCoriolisAcceleration(desiredLinearAcceleration);

      inverseDynamicsOutput.setSpatialAcceleration(controlFrame, desiredAngularAcceleration, desiredLinearAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);

      yoFeedForwardVelocity.getIncludingFrame(feedForwardLinearVelocity, feedForwardAngularVelocity);
      computeProportionalTerm(linearProportionalFeedback, angularProportionalFeedback);
      computeIntegralTerm(linearIntegralFeedback, angularIntegralFeedback);

      desiredLinearVelocity.setIncludingFrame(linearProportionalFeedback);
      desiredLinearVelocity.add(linearIntegralFeedback);
      desiredLinearVelocity.clipToMaxLength(positionGains.getMaximumFeedback());

      desiredAngularVelocity.setIncludingFrame(angularProportionalFeedback);
      desiredAngularVelocity.add(angularIntegralFeedback);
      desiredAngularVelocity.clipToMaxLength(orientationGains.getMaximumFeedback());

      yoFeedbackVelocity.setAndMatchFrame(desiredLinearVelocity, desiredAngularVelocity);
      rateLimitedFeedbackVelocity.update();
      rateLimitedFeedbackVelocity.getIncludingFrame(desiredLinearVelocity, desiredAngularVelocity);

      desiredLinearVelocity.add(feedForwardLinearVelocity);
      desiredAngularVelocity.add(feedForwardAngularVelocity);

      yoDesiredVelocity.setAndMatchFrame(desiredLinearVelocity, desiredAngularVelocity);

      desiredLinearVelocity.changeFrame(controlFrame);
      desiredAngularVelocity.changeFrame(controlFrame);
      inverseKinematicsOutput.setSpatialVelocity(controlFrame, desiredAngularVelocity, desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      if (!isEnabled())
         return;

      computeFeedbackWrench();

      if (endEffector.getName().equals(rootBody.getName()))
      {
         desiredLinearForce.changeFrame(worldFrame);
         desiredAngularTorque.changeFrame(worldFrame);

         virtualModelControlRootOutput.setProperties(inverseDynamicsOutput);
         virtualModelControlRootOutput.setMomentumRate(desiredAngularTorque, desiredLinearForce);
      }
      else
      {
         virtualModelControlOutput.setProperties(inverseDynamicsOutput);
         virtualModelControlOutput.setWrench(controlFrame, desiredAngularTorque, desiredLinearForce);
      }
   }

   private void computeFeedbackWrench()
   {
      yoFeedForwardWrench.getIncludingFrame(feedForwardLinearAction, feedForwardAngularAction);
      feedForwardLinearAction.changeFrame(controlFrame);
      feedForwardAngularAction.changeFrame(controlFrame);
      computeProportionalTerm(linearProportionalFeedback, angularProportionalFeedback);
      computeDerivativeTerm(linearDerivativeFeedback, angularDerivativeFeedback);
      computeIntegralTerm(linearIntegralFeedback, angularIntegralFeedback);

      desiredLinearForce.setIncludingFrame(linearProportionalFeedback);
      desiredLinearForce.add(linearDerivativeFeedback);
      desiredLinearForce.add(linearIntegralFeedback);
      desiredLinearForce.clipToMaxLength(positionGains.getMaximumFeedback());

      desiredAngularTorque.setIncludingFrame(angularProportionalFeedback);
      desiredAngularTorque.add(angularDerivativeFeedback);
      desiredAngularTorque.add(angularIntegralFeedback);
      desiredAngularTorque.clipToMaxLength(orientationGains.getMaximumFeedback());

      yoFeedbackWrench.setAndMatchFrame(desiredLinearForce, desiredAngularTorque);
      rateLimitedFeedbackWrench.update();
      rateLimitedFeedbackWrench.getIncludingFrame(desiredLinearForce, desiredAngularTorque);

      desiredLinearForce.changeFrame(controlFrame);
      desiredLinearForce.add(feedForwardLinearAction);

      desiredAngularTorque.changeFrame(controlFrame);
      desiredAngularTorque.add(feedForwardAngularAction);

      yoDesiredWrench.setAndMatchFrame(desiredLinearForce, desiredAngularTorque);
   }

   @Override
   public void computeAchievedAcceleration()
   {
      spatialAccelerationCalculator.getRelativeAcceleration(base, endEffector, endEffectorAchievedAcceleration);
      endEffectorAchievedAcceleration.changeFrameNoRelativeMotion(controlFrame);
      endEffectorAchievedAcceleration.getAngularPart(achievedAngularAcceleration);
      endEffectorAchievedAcceleration.getLinearPart(achievedLinearAcceleration);
      subtractCoriolisAcceleration(achievedLinearAcceleration);

      yoAchievedAcceleration.setAndMatchFrame(achievedLinearAcceleration, achievedAngularAcceleration);
   }

   /**
    * Computes the feedback term resulting from the error in position and orienation:<br>
    * x<sub>FB</sub><sup>linear</sup> = kp<sup>linear</sup> * (x<sub>desired</sub> -
    * x<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = kp<sup>angular</sup> * &theta;<sub>error</sub><br>
    * where &theta;<sub>error</sub> is a rotation vector representing the current error in
    * orientation.
    * <p>
    * The desired pose of the {@code controlFrame} is obtained from {@link #yoDesiredPose}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentPose}, {@link #yoErrorVector}, and
    * {@link #yoErrorOrientation}.
    * </p>
    *
    * @param linearFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>linear</sup>. Modified.
    * @param angularFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>angular</sup>. Modified.
    */
   private void computeProportionalTerm(FrameVector3D linearFeedbackTermToPack, FrameVector3D angularFeedbackTermToPack)
   {
      currentPose.setToZero(controlFrame);
      currentPose.changeFrame(worldFrame);
      yoCurrentPose.set(currentPose);
      yoCurrentPose.getOrientation().getRotationVector(yoCurrentRotationVector);

      desiredPose.setIncludingFrame(yoDesiredPose);
      desiredPose.changeFrame(controlFrame);

      desiredPose.normalizeQuaternionAndLimitToPi();
      linearFeedbackTermToPack.setIncludingFrame(desiredPose.getPosition());
      desiredPose.getRotationVector(angularFeedbackTermToPack);

      selectionMatrix.applyLinearSelection(linearFeedbackTermToPack);
      selectionMatrix.applyAngularSelection(angularFeedbackTermToPack);

      linearFeedbackTermToPack.clipToMaxLength(positionGains.getMaximumProportionalError());
      angularFeedbackTermToPack.clipToMaxLength(orientationGains.getMaximumProportionalError());

      yoErrorVector.setAndMatchFrame(linearFeedbackTermToPack, angularFeedbackTermToPack);
      yoErrorOrientation.setRotationVector(yoErrorVector.getYoAngularPart());

      if (linearGainsFrame != null)
         linearFeedbackTermToPack.changeFrame(linearGainsFrame);
      else
         linearFeedbackTermToPack.changeFrame(controlFrame);

      if (angularGainsFrame != null)
         angularFeedbackTermToPack.changeFrame(angularGainsFrame);
      else
         angularFeedbackTermToPack.changeFrame(controlFrame);

      positionGains.getProportionalGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(linearFeedbackTermToPack);

      orientationGains.getProportionalGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(angularFeedbackTermToPack);

      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the error in linear velocity:<br>
    * x<sub>FB</sub><sup>linear</sup> = kd<sup>linear</sup> * (xDot<sub>desired</sub> -
    * xDot<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = kd<sup>angular</sup> * (&omega;<sub>desired</sub> -
    * &omega;<sub>current</sub>)
    * <p>
    * The desired velocity of the {@code controlFrame} relative to the {@code base} is obtained from
    * {@link #yoDesiredVelocity}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentVelocity} and {@link #yoErrorVelocity}.
    * </p>
    *
    * @param linearFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>linear</sup>. Modified.
    * @param angularFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>angular</sup>. Modified.
    */
   private void computeDerivativeTerm(FrameVector3D linearFeedbackTermToPack, FrameVector3D angularFeedbackTermToPack)
   {
      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      currentTwist.getLinearPart(currentLinearVelocity);
      currentTwist.getAngularPart(currentAngularVelocity);
      currentLinearVelocity.changeFrame(worldFrame);
      currentAngularVelocity.changeFrame(worldFrame);
      yoCurrentVelocity.setAndMatchFrame(currentLinearVelocity, currentAngularVelocity);

      yoDesiredVelocity.getIncludingFrame(desiredLinearVelocity, desiredAngularVelocity);

      linearFeedbackTermToPack.setToZero(worldFrame);
      angularFeedbackTermToPack.setToZero(worldFrame);
      linearFeedbackTermToPack.sub(desiredLinearVelocity, currentLinearVelocity);
      angularFeedbackTermToPack.sub(desiredAngularVelocity, currentAngularVelocity);

      selectionMatrix.applyLinearSelection(linearFeedbackTermToPack);
      selectionMatrix.applyAngularSelection(angularFeedbackTermToPack);

      linearFeedbackTermToPack.clipToMaxLength(positionGains.getMaximumDerivativeError());
      angularFeedbackTermToPack.clipToMaxLength(orientationGains.getMaximumDerivativeError());
      yoErrorVelocity.set(linearFeedbackTermToPack, angularFeedbackTermToPack);

      if (linearGainsFrame != null)
         linearFeedbackTermToPack.changeFrame(linearGainsFrame);
      else
         linearFeedbackTermToPack.changeFrame(controlFrame);

      if (angularGainsFrame != null)
         angularFeedbackTermToPack.changeFrame(angularGainsFrame);
      else
         angularFeedbackTermToPack.changeFrame(controlFrame);

      positionGains.getDerivativeGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(linearFeedbackTermToPack);

      orientationGains.getDerivativeGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(angularFeedbackTermToPack);

      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the integrated error in position:<br>
    * x<sub>FB</sub><sup>linear</sup> = ki<sup>linear</sup> * &int;<sup>t</sup> (x<sub>desired</sub>
    * - x<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = ki<sup>angular</sup> * &int;<sup>t</sup>
    * &theta;<sub>error</sub>
    * <p>
    * The current error in position and orientation of the {@code controlFrame} is obtained from
    * {@link #yoErrorVector} and {@link #yoErrorOrientation} respectively.
    * </p>
    * <p>
    * This method also updates {@link #yoErrorPositionIntegrated},
    * {@link #yoErrorOrientationCumulated}, and {@link #yoErrorRotationVectorIntegrated}.
    * </p>
    *
    * @param linearFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>linear</sup>. Modified.
    * @param angularFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>angular</sup>. Modified.
    */
   private void computeIntegralTerm(FrameVector3D linearFeedbackTermToPack, FrameVector3D angularFeedbackTermToPack)
   {
      double maximumLinearIntegralError = positionGains.getMaximumIntegralError();

      if (maximumLinearIntegralError < 1.0e-5)
      {
         linearFeedbackTermToPack.setToZero(controlFrame);
         yoErrorPositionIntegrated.setToZero();
      }
      else
      {
         yoErrorVector.getLinearPartIncludingFrame(linearFeedbackTermToPack);
         linearFeedbackTermToPack.scale(dt);
         linearFeedbackTermToPack.add(yoErrorPositionIntegrated);
         selectionMatrix.applyLinearSelection(linearFeedbackTermToPack);
         linearFeedbackTermToPack.clipToMaxLength(maximumLinearIntegralError);
         yoErrorPositionIntegrated.set(linearFeedbackTermToPack);

         if (linearGainsFrame != null)
            linearFeedbackTermToPack.changeFrame(linearGainsFrame);
         else
            linearFeedbackTermToPack.changeFrame(controlFrame);

         positionGains.getIntegralGainMatrix(tempGainMatrix);
         tempGainMatrix.transform(linearFeedbackTermToPack);

         linearFeedbackTermToPack.changeFrame(controlFrame);
      }

      double maximumAngularIntegralError = orientationGains.getMaximumIntegralError();

      if (maximumAngularIntegralError < 1.0e-5)
      {
         angularFeedbackTermToPack.setToZero(controlFrame);
         yoErrorOrientationCumulated.setToZero();
         yoErrorRotationVectorIntegrated.setToZero();
      }
      else
      {
         errorOrientationCumulated.setIncludingFrame(yoErrorOrientationCumulated);
         errorOrientationCumulated.multiply(yoErrorOrientation);
         yoErrorOrientationCumulated.set(errorOrientationCumulated);
         errorOrientationCumulated.normalizeAndLimitToPi();

         errorOrientationCumulated.getRotationVector(angularFeedbackTermToPack);
         angularFeedbackTermToPack.scale(dt);
         selectionMatrix.applyAngularSelection(angularFeedbackTermToPack);
         angularFeedbackTermToPack.clipToMaxLength(maximumAngularIntegralError);
         yoErrorRotationVectorIntegrated.set(angularFeedbackTermToPack);

         if (angularGainsFrame != null)
            angularFeedbackTermToPack.changeFrame(angularGainsFrame);
         else
            angularFeedbackTermToPack.changeFrame(controlFrame);

         orientationGains.getIntegralGainMatrix(tempGainMatrix);
         tempGainMatrix.transform(angularFeedbackTermToPack);

         angularFeedbackTermToPack.changeFrame(controlFrame);
      }
   }

   /**
    * Computes and adds the bias acceleration resulting from the combination of the current linear
    * and angular velocity of the control frame.
    * <p>
    * This is needed when going from a linear acceleration expressed in an inertial frame to a
    * moving frame attached to the end-effector.
    * </p>
    * <p>
    * Intuitively, the Coriolis acceleration only appears when measuring the acceleration from a
    * moving frame, here a frame attache to the end-effector.
    * </p>
    *
    * @param linearAccelerationToModify the linear acceleration vector to which the bias is to be
    *           subtracted. Its frame is changed to {@code controlFrame}. Modified.
    */
   private void addCoriolisAcceleration(FrameVector3D linearAccelerationToModify)
   {
      controlFrame.getTwistOfFrame(currentTwist);
      currentTwist.getAngularPart(currentAngularVelocity);
      currentTwist.getLinearPart(currentLinearVelocity);

      biasLinearAcceleration.setToZero(controlFrame);
      biasLinearAcceleration.cross(currentLinearVelocity, currentAngularVelocity);
      linearAccelerationToModify.changeFrame(controlFrame);
      linearAccelerationToModify.add(biasLinearAcceleration);
   }

   /**
    * Computes and subtracts the bias acceleration resulting from the combination of the current
    * linear and angular velocity of the control frame.
    * <p>
    * This is needed when going from a linear acceleration expressed in a moving frame attached to
    * the end-effector to an inertial frame.
    * </p>
    * <p>
    * Intuitively, the Coriolis acceleration only appears when measuring the acceleration from a
    * moving frame, here a frame attache to the end-effector.
    * </p>
    *
    * @param linearAccelerationToModify the linear acceleration vector to which the bias is to be
    *           added. Its frame is changed to {@code worldFrame}. Modified.
    */
   private void subtractCoriolisAcceleration(FrameVector3D linearAccelerationToModify)
   {
      controlFrame.getTwistOfFrame(currentTwist);
      currentTwist.getAngularPart(currentAngularVelocity);
      currentTwist.getLinearPart(currentLinearVelocity);

      biasLinearAcceleration.setToZero(controlFrame);
      biasLinearAcceleration.cross(currentLinearVelocity, currentAngularVelocity);
      linearAccelerationToModify.changeFrame(controlFrame);
      linearAccelerationToModify.sub(biasLinearAcceleration);
      linearAccelerationToModify.changeFrame(worldFrame);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public SpatialAccelerationCommand getInverseDynamicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseDynamicsOutput;
   }

   @Override
   public SpatialVelocityCommand getInverseKinematicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseKinematicsOutput;
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return (endEffector.getName().equals(rootBody.getName())) ? virtualModelControlRootOutput : virtualModelControlOutput;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": endEffector = " + endEffector;
   }
}
