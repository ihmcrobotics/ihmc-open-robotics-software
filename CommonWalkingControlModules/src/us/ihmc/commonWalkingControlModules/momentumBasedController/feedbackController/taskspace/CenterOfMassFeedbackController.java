package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.*;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.*;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.CentroidalMomentumHandler;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public class CenterOfMassFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String shortName = "CenterOfMassFBController";
   private final YoVariableRegistry registry = new YoVariableRegistry(shortName);

   private final YoBoolean isEnabled = new YoBoolean("is" + shortName + "Enabled", registry);

   private final YoFramePoint yoDesiredPosition;
   private final YoFramePoint yoCurrentPosition;
   private final YoFrameVector yoErrorPosition;

   private final YoFrameVector yoErrorPositionIntegrated;

   private final YoFrameVector yoDesiredLinearVelocity;
   private final YoFrameVector yoCurrentLinearVelocity;
   private final YoFrameVector yoErrorLinearVelocity;
   private final YoFrameVector yoFeedForwardLinearVelocity;
   private final YoFrameVector yoFeedbackLinearVelocity;
   private final RateLimitedYoFrameVector rateLimitedFeedbackLinearVelocity;

   private final YoFrameVector yoDesiredLinearAcceleration;
   private final YoFrameVector yoFeedForwardLinearAcceleration;
   private final YoFrameVector yoFeedbackLinearAcceleration;
   private final RateLimitedYoFrameVector rateLimitedFeedbackLinearAcceleration;
   private final YoFrameVector yoAchievedLinearAcceleration;

   private final FramePoint desiredPosition = new FramePoint();
   private final FramePoint currentPosition = new FramePoint();

   private final FrameVector desiredLinearVelocity = new FrameVector();
   private final FrameVector currentLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearVelocity = new FrameVector();

   private final FrameVector achievedLinearAcceleration = new FrameVector();
   private final FrameVector desiredLinearAcceleration = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();

   private final MomentumRateCommand inverseDynamicsOutput = new MomentumRateCommand();
   private final MomentumCommand inverseKinematicsOutput = new MomentumCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final YoPositionPIDGainsInterface gains;
   private final Matrix3DReadOnly kp, kd, ki;

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
      kp = gains.createProportionalGainMatrix();
      kd = gains.createDerivativeGainMatrix();
      ki = gains.createIntegralGainMatrix();
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
      yoDesiredPosition.setAndMatchFrame(desiredPosition);
      yoDesiredLinearVelocity.setAndMatchFrame(desiredLinearVelocity);
      if (yoFeedForwardLinearVelocity != null)
         yoFeedForwardLinearVelocity.setAndMatchFrame(desiredLinearVelocity);
      if (yoFeedForwardLinearAcceleration != null)
         yoFeedForwardLinearAcceleration.setAndMatchFrame(feedForwardLinearAcceleration);
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

   private final FrameVector proportionalFeedback = new FrameVector();
   private final FrameVector derivativeFeedback = new FrameVector();
   private final FrameVector integralFeedback = new FrameVector();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);
      yoFeedForwardLinearAcceleration.getFrameTupleIncludingFrame(feedForwardLinearAcceleration);
      feedForwardLinearAcceleration.changeFrame(centerOfMassFrame);

      desiredLinearAcceleration.setIncludingFrame(proportionalFeedback);
      desiredLinearAcceleration.add(derivativeFeedback);
      desiredLinearAcceleration.add(integralFeedback);
      desiredLinearAcceleration.limitLength(gains.getMaximumFeedback());
      yoFeedbackLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);
      rateLimitedFeedbackLinearAcceleration.update();
      rateLimitedFeedbackLinearAcceleration.getFrameTupleIncludingFrame(desiredLinearAcceleration);

      desiredLinearAcceleration.changeFrame(centerOfMassFrame);
      desiredLinearAcceleration.add(feedForwardLinearAcceleration);

      yoDesiredLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);

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

      yoFeedForwardLinearVelocity.getFrameTupleIncludingFrame(feedForwardLinearVelocity);
      computeProportionalTerm(proportionalFeedback);
      computeIntegralTerm(integralFeedback);

      desiredLinearVelocity.setIncludingFrame(proportionalFeedback);
      desiredLinearVelocity.add(integralFeedback);
      desiredLinearVelocity.limitLength(gains.getMaximumFeedback());
      yoFeedbackLinearVelocity.setAndMatchFrame(desiredLinearVelocity);
      rateLimitedFeedbackLinearVelocity.update();
      rateLimitedFeedbackLinearVelocity.getFrameTupleIncludingFrame(desiredLinearVelocity);

      desiredLinearVelocity.add(feedForwardLinearVelocity);

      yoDesiredLinearVelocity.setAndMatchFrame(desiredLinearVelocity);

      desiredLinearVelocity.scale(totalRobotMass);
      desiredLinearVelocity.changeFrame(worldFrame);
      inverseKinematicsOutput.setLinearMomentum(desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      computeInverseDynamics();
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
   private void computeProportionalTerm(FrameVector feedbackTermToPack)
   {
      currentPosition.setToZero(centerOfMassFrame);
      currentPosition.changeFrame(worldFrame);
      yoCurrentPosition.set(currentPosition);

      yoDesiredPosition.getFrameTupleIncludingFrame(desiredPosition);

      feedbackTermToPack.setToZero(worldFrame);
      feedbackTermToPack.sub(desiredPosition, currentPosition);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.limitLength(gains.getMaximumProportionalError());
      yoErrorPosition.set(feedbackTermToPack);

      feedbackTermToPack.changeFrame(centerOfMassFrame);
      kp.transform(feedbackTermToPack.getVector());
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
   private void computeDerivativeTerm(FrameVector feedbackTermToPack)
   {
      centroidalMomentumHandler.getCenterOfMassVelocity(currentLinearVelocity);
      currentLinearVelocity.changeFrame(worldFrame);
      yoCurrentLinearVelocity.set(currentLinearVelocity);

      yoDesiredLinearVelocity.getFrameTupleIncludingFrame(desiredLinearVelocity);

      feedbackTermToPack.setToZero(worldFrame);
      feedbackTermToPack.sub(desiredLinearVelocity, currentLinearVelocity);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.limitLength(gains.getMaximumDerivativeError());
      yoErrorLinearVelocity.set(feedbackTermToPack);

      feedbackTermToPack.changeFrame(centerOfMassFrame);
      kd.transform(feedbackTermToPack.getVector());
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
   private void computeIntegralTerm(FrameVector feedbackTermToPack)
   {
      double maximumIntegralError = gains.getMaximumIntegralError();

      if (maximumIntegralError < 1.0e-5)
      {
         feedbackTermToPack.setToZero(centerOfMassFrame);
         yoErrorPositionIntegrated.setToZero();
         return;
      }

      yoErrorPosition.getFrameTupleIncludingFrame(feedbackTermToPack);
      feedbackTermToPack.scale(dt);
      feedbackTermToPack.add(yoErrorPositionIntegrated.getFrameTuple());
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.limitLength(maximumIntegralError);
      yoErrorPositionIntegrated.set(feedbackTermToPack);

      feedbackTermToPack.changeFrame(centerOfMassFrame);
      ki.transform(feedbackTermToPack.getVector());
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
   public InverseDynamicsCommand<?> getVirtualModelControlOutput()
   {
      return getInverseDynamicsOutput();
   }
}
