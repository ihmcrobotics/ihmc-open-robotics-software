package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.LINEAR_ACCELERATION;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.LINEAR_VELOCITY;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.POSITION;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ACHIEVED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.CURRENT;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.DESIRED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ERROR;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ERROR_INTEGRATED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.FEEDBACK;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.FEEDFORWARD;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.CentroidalMomentumHandler;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class CenterOfMassFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String shortName = "CenterOfMassFBController";
   private final YoVariableRegistry registry = new YoVariableRegistry(shortName);

   private final YoBoolean isEnabled = new YoBoolean("is" + shortName + "Enabled", registry);

   private final YoFramePoint3D yoDesiredPosition;
   private final YoFramePoint3D yoCurrentPosition;
   private final YoFrameVector3D yoErrorPosition;

   private final YoFrameVector3D yoErrorPositionIntegrated;

   private final YoFrameVector3D yoDesiredLinearVelocity;
   private final YoFrameVector3D yoCurrentLinearVelocity;
   private final YoFrameVector3D yoErrorLinearVelocity;
   private final YoFrameVector3D yoFeedForwardLinearVelocity;
   private final YoFrameVector3D yoFeedbackLinearVelocity;
   private final RateLimitedYoFrameVector rateLimitedFeedbackLinearVelocity;

   private final YoFrameVector3D yoDesiredLinearAcceleration;
   private final YoFrameVector3D yoFeedForwardLinearAcceleration;
   private final YoFrameVector3D yoFeedbackLinearAcceleration;
   private final RateLimitedYoFrameVector rateLimitedFeedbackLinearAcceleration;
   private final YoFrameVector3D yoAchievedLinearAcceleration;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FramePoint3D currentPosition = new FramePoint3D();

   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D currentLinearVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearVelocity = new FrameVector3D();

   private final FrameVector3D achievedLinearAcceleration = new FrameVector3D();
   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();

   private final MomentumRateCommand inverseDynamicsOutput = new MomentumRateCommand();
   private final MomentumCommand inverseKinematicsOutput = new MomentumCommand();
   private final MomentumRateCommand virtualModelControlOutput = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final YoPID3DGains gains;
   private final Matrix3D tempGainMatrix = new Matrix3D();

   private ReferenceFrame centerOfMassFrame;
   private CentroidalMomentumHandler centroidalMomentumHandler;

   private final double dt;
   private final double totalRobotMass;

   public CenterOfMassFeedbackController(WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
                                         YoVariableRegistry parentRegistry)
   {
      centerOfMassFrame = toolbox.getCenterOfMassFrame();
      centroidalMomentumHandler = toolbox.getCentroidalMomentumHandler();
      totalRobotMass = toolbox.getTotalRobotMass();

      dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getCenterOfMassGains();
      YoDouble maximumRate = gains.getYoMaximumFeedbackRate();

      isEnabled.set(false);

      yoDesiredPosition = feedbackControllerToolbox.getCenterOfMassPosition(DESIRED, isEnabled);
      yoCurrentPosition = feedbackControllerToolbox.getCenterOfMassPosition(CURRENT, isEnabled);
      yoErrorPosition = feedbackControllerToolbox.getCenterOfMassDataVector(ERROR, POSITION, isEnabled);

      yoErrorPositionIntegrated = feedbackControllerToolbox.getCenterOfMassDataVector(ERROR_INTEGRATED, POSITION, isEnabled);

      yoDesiredLinearVelocity = feedbackControllerToolbox.getCenterOfMassDataVector(DESIRED, LINEAR_VELOCITY, isEnabled);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentLinearVelocity = feedbackControllerToolbox.getCenterOfMassDataVector(CURRENT, LINEAR_VELOCITY, isEnabled);
         yoErrorLinearVelocity = feedbackControllerToolbox.getCenterOfMassDataVector(ERROR, LINEAR_VELOCITY, isEnabled);

         yoDesiredLinearAcceleration = feedbackControllerToolbox.getCenterOfMassDataVector(DESIRED, LINEAR_ACCELERATION, isEnabled);
         yoFeedForwardLinearAcceleration = feedbackControllerToolbox.getCenterOfMassDataVector(FEEDFORWARD, LINEAR_ACCELERATION, isEnabled);
         yoFeedbackLinearAcceleration = feedbackControllerToolbox.getCenterOfMassDataVector(FEEDBACK, LINEAR_ACCELERATION, isEnabled);
         rateLimitedFeedbackLinearAcceleration = feedbackControllerToolbox.getCenterOfMassRateLimitedDataVector(FEEDBACK, LINEAR_ACCELERATION, dt, maximumRate,
                                                                                                                isEnabled);
         yoAchievedLinearAcceleration = feedbackControllerToolbox.getCenterOfMassDataVector(ACHIEVED, LINEAR_ACCELERATION, isEnabled);
      }
      else
      {
         yoCurrentLinearVelocity = null;
         yoErrorLinearVelocity = null;

         yoDesiredLinearAcceleration = null;
         yoFeedForwardLinearAcceleration = null;
         yoFeedbackLinearAcceleration = null;
         rateLimitedFeedbackLinearAcceleration = null;
         yoAchievedLinearAcceleration = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackLinearVelocity = feedbackControllerToolbox.getCenterOfMassDataVector(FEEDBACK, LINEAR_VELOCITY, isEnabled);
         yoFeedForwardLinearVelocity = feedbackControllerToolbox.getCenterOfMassDataVector(FEEDFORWARD, LINEAR_VELOCITY, isEnabled);
         rateLimitedFeedbackLinearVelocity = feedbackControllerToolbox.getCenterOfMassRateLimitedDataVector(FEEDBACK, LINEAR_VELOCITY, dt, maximumRate,
                                                                                                            isEnabled);
      }
      else
      {
         yoFeedbackLinearVelocity = null;
         yoFeedForwardLinearVelocity = null;
         rateLimitedFeedbackLinearVelocity = null;
      }

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(CenterOfMassFeedbackControlCommand command)
   {
      inverseDynamicsOutput.set(command.getMomentumRateCommand());

      gains.set(command.getGains());
      command.getMomentumRateCommand().getSelectionMatrix(selectionMatrix);

      command.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      yoDesiredPosition.setMatchingFrame(desiredPosition);
      yoDesiredLinearVelocity.setMatchingFrame(desiredLinearVelocity);
      if (yoFeedForwardLinearVelocity != null)
         yoFeedForwardLinearVelocity.setMatchingFrame(desiredLinearVelocity);
      if (yoFeedForwardLinearAcceleration != null)
         yoFeedForwardLinearAcceleration.setMatchingFrame(feedForwardLinearAcceleration);
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   {
      if (rateLimitedFeedbackLinearAcceleration != null)
         rateLimitedFeedbackLinearAcceleration.reset();
      if (rateLimitedFeedbackLinearVelocity != null)
         rateLimitedFeedbackLinearVelocity.reset();
   }

   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();
   private final FrameVector3D integralFeedback = new FrameVector3D();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);
      feedForwardLinearAcceleration.setIncludingFrame(yoFeedForwardLinearAcceleration);
      feedForwardLinearAcceleration.changeFrame(centerOfMassFrame);

      desiredLinearAcceleration.setIncludingFrame(proportionalFeedback);
      desiredLinearAcceleration.add(derivativeFeedback);
      desiredLinearAcceleration.add(integralFeedback);
      desiredLinearAcceleration.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackLinearAcceleration.setMatchingFrame(desiredLinearAcceleration);
      rateLimitedFeedbackLinearAcceleration.update();
      desiredLinearAcceleration.setIncludingFrame(rateLimitedFeedbackLinearAcceleration);

      desiredLinearAcceleration.changeFrame(centerOfMassFrame);
      desiredLinearAcceleration.add(feedForwardLinearAcceleration);

      yoDesiredLinearAcceleration.setMatchingFrame(desiredLinearAcceleration);

      desiredLinearAcceleration.scale(totalRobotMass);
      desiredLinearAcceleration.changeFrame(worldFrame);
      inverseDynamicsOutput.setLinearMomentumRate(desiredLinearAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);

      feedForwardLinearVelocity.setIncludingFrame(yoFeedForwardLinearVelocity);
      computeProportionalTerm(proportionalFeedback);
      computeIntegralTerm(integralFeedback);

      desiredLinearVelocity.setIncludingFrame(proportionalFeedback);
      desiredLinearVelocity.add(integralFeedback);
      desiredLinearVelocity.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackLinearVelocity.setMatchingFrame(desiredLinearVelocity);
      rateLimitedFeedbackLinearVelocity.update();
      desiredLinearVelocity.setIncludingFrame(rateLimitedFeedbackLinearVelocity);

      desiredLinearVelocity.add(feedForwardLinearVelocity);

      yoDesiredLinearVelocity.setMatchingFrame(desiredLinearVelocity);

      desiredLinearVelocity.scale(totalRobotMass);
      desiredLinearVelocity.changeFrame(worldFrame);
      inverseKinematicsOutput.setLinearMomentum(desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      if (!isEnabled())
         return;

      virtualModelControlOutput.set(inverseDynamicsOutput);

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);
      feedForwardLinearAcceleration.setIncludingFrame(yoFeedForwardLinearAcceleration);
      feedForwardLinearAcceleration.changeFrame(centerOfMassFrame);

      desiredLinearAcceleration.setIncludingFrame(proportionalFeedback);
      desiredLinearAcceleration.add(derivativeFeedback);
      desiredLinearAcceleration.add(integralFeedback);
      desiredLinearAcceleration.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackLinearAcceleration.setMatchingFrame(desiredLinearAcceleration);
      rateLimitedFeedbackLinearAcceleration.update();
      desiredLinearAcceleration.setIncludingFrame(rateLimitedFeedbackLinearAcceleration);

      desiredLinearAcceleration.changeFrame(centerOfMassFrame);
      desiredLinearAcceleration.add(feedForwardLinearAcceleration);

      yoDesiredLinearAcceleration.setMatchingFrame(desiredLinearAcceleration);

      desiredLinearAcceleration.scale(totalRobotMass);
      desiredLinearAcceleration.changeFrame(worldFrame);
      virtualModelControlOutput.setLinearMomentumRate(desiredLinearAcceleration);
   }

   @Override
   public void computeAchievedAcceleration()
   {
      SpatialForceVector achievedMomentumRate = centroidalMomentumHandler.getCentroidalMomentumRate();
      achievedMomentumRate.getLinearPartIncludingFrame(achievedLinearAcceleration);
      achievedLinearAcceleration.changeFrame(worldFrame);
      achievedLinearAcceleration.scale(1.0 / totalRobotMass);
      yoAchievedLinearAcceleration.set(achievedLinearAcceleration);
   }

   /**
    * Computes the feedback term resulting from the error in position:<br>
    * x<sub>FB</sub> = kp * (x<sub>desired</sub> - x<sub>current</sub>)
    * <p>
    * The desired center of mass position is obtained from {@link #yoDesiredPosition}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentPosition} and {@link #yoErrorPosition}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   private void computeProportionalTerm(FrameVector3D feedbackTermToPack)
   {
      currentPosition.setToZero(centerOfMassFrame);
      currentPosition.changeFrame(worldFrame);
      yoCurrentPosition.set(currentPosition);

      desiredPosition.setIncludingFrame(yoDesiredPosition);

      feedbackTermToPack.setToZero(worldFrame);
      feedbackTermToPack.sub(desiredPosition, currentPosition);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumProportionalError());
      yoErrorPosition.set(feedbackTermToPack);

      feedbackTermToPack.changeFrame(centerOfMassFrame);
      gains.getProportionalGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);
   }

   /**
    * Computes the feedback term resulting from the error in linear velocity:<br>
    * x<sub>FB</sub> = kd * (xDot<sub>desired</sub> - xDot<sub>current</sub>)
    * <p>
    * The desired center of mass velocity linear velocity with respect to {@code worldFrame} is
    * obtained from {@link #yoDesiredLinearVelocity}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentLinearVelocity} and {@link #yoErrorLinearVelocity}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified
    */
   private void computeDerivativeTerm(FrameVector3D feedbackTermToPack)
   {
      centroidalMomentumHandler.getCenterOfMassLinearVelocity(currentLinearVelocity);
      currentLinearVelocity.changeFrame(worldFrame);
      yoCurrentLinearVelocity.set(currentLinearVelocity);

      desiredLinearVelocity.setIncludingFrame(yoDesiredLinearVelocity);

      feedbackTermToPack.setToZero(worldFrame);
      feedbackTermToPack.sub(desiredLinearVelocity, currentLinearVelocity);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumDerivativeError());
      yoErrorLinearVelocity.set(feedbackTermToPack);

      feedbackTermToPack.changeFrame(centerOfMassFrame);
      gains.getDerivativeGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);
   }

   /**
    * Computes the feedback term resulting from the integrated error in position:<br>
    * x<sub>FB</sub> = ki * &int;<sup>t</sup> (x<sub>desired</sub> - x<sub>current</sub>)
    * <p>
    * The current error in position of the center of mass is obtained from {@link #yoErrorPosition}.
    * </p>
    * <p>
    * This method also updates {@link #yoErrorPositionIntegrated}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   private void computeIntegralTerm(FrameVector3D feedbackTermToPack)
   {
      double maximumIntegralError = gains.getMaximumIntegralError();

      if (maximumIntegralError < 1.0e-5)
      {
         feedbackTermToPack.setToZero(centerOfMassFrame);
         yoErrorPositionIntegrated.setToZero();
         return;
      }

      feedbackTermToPack.setIncludingFrame(yoErrorPosition);
      feedbackTermToPack.scale(dt);
      feedbackTermToPack.add(yoErrorPositionIntegrated);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(maximumIntegralError);
      yoErrorPositionIntegrated.set(feedbackTermToPack);

      feedbackTermToPack.changeFrame(centerOfMassFrame);
      gains.getIntegralGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public MomentumRateCommand getInverseDynamicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseDynamicsOutput;
   }

   @Override
   public MomentumCommand getInverseKinematicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseKinematicsOutput;
   }

   @Override
   public MomentumRateCommand getVirtualModelControlOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return virtualModelControlOutput;
   }
}
