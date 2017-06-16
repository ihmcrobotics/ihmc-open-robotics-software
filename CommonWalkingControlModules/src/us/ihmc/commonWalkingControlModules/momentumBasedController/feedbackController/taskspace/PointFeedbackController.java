package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.*;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.*;

import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
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
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;

public class PointFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoBoolean isEnabled;

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
   private final FrameVector currentAngularVelocity = new FrameVector();
   private final FrameVector feedForwardLinearVelocity = new FrameVector();

   private final FrameVector desiredLinearAcceleration = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector biasLinearAcceleration = new FrameVector();
   private final FrameVector achievedLinearAcceleration = new FrameVector();

   private final Twist currentTwist = new Twist();

   private final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();

   private final YoPositionPIDGainsInterface gains;
   private final Matrix3DReadOnly kp, kd, ki;
   private final YoSE3OffsetFrame controlFrame;

   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private RigidBody base;
   private ReferenceFrame controlBaseFrame;
   private ReferenceFrame linearGainsFrame;

   private final RigidBody endEffector;

   private final double dt;

   public PointFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
                                  YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;

      spatialAccelerationCalculator = toolbox.getSpatialAccelerationCalculator();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "PointFBController");
      dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getPositionGains(endEffector);
      kp = gains.createProportionalGainMatrix();
      kd = gains.createDerivativeGainMatrix();
      ki = gains.createIntegralGainMatrix();
      YoDouble maximumRate = gains.getYoMaximumFeedbackRate();

      controlFrame = feedbackControllerToolbox.getControlFrame(endEffector);

      isEnabled = new YoBoolean(endEffectorName + "isPointFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredPosition = feedbackControllerToolbox.getPosition(endEffector, DESIRED, isEnabled);
      yoCurrentPosition = feedbackControllerToolbox.getPosition(endEffector, CURRENT, isEnabled);
      yoErrorPosition = feedbackControllerToolbox.getDataVector(endEffector, ERROR, POSITION, isEnabled);

      yoErrorPositionIntegrated = feedbackControllerToolbox.getDataVector(endEffector, ERROR_INTEGRATED, POSITION, isEnabled);

      yoDesiredLinearVelocity = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, LINEAR_VELOCITY, isEnabled);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentLinearVelocity = feedbackControllerToolbox.getDataVector(endEffector, CURRENT, LINEAR_VELOCITY, isEnabled);
         yoErrorLinearVelocity = feedbackControllerToolbox.getDataVector(endEffector, ERROR, LINEAR_VELOCITY, isEnabled);

         yoDesiredLinearAcceleration = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, LINEAR_ACCELERATION, isEnabled);
         yoFeedForwardLinearAcceleration = feedbackControllerToolbox.getDataVector(endEffector, FEEDFORWARD, LINEAR_ACCELERATION, isEnabled);
         yoFeedbackLinearAcceleration = feedbackControllerToolbox.getDataVector(endEffector, Type.FEEDBACK, LINEAR_ACCELERATION, isEnabled);
         rateLimitedFeedbackLinearAcceleration = feedbackControllerToolbox.getRateLimitedDataVector(endEffector, FEEDBACK, LINEAR_ACCELERATION, dt, maximumRate,
                                                                                                    isEnabled);
         yoAchievedLinearAcceleration = feedbackControllerToolbox.getDataVector(endEffector, Type.ACHIEVED, LINEAR_ACCELERATION, isEnabled);
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
         yoFeedbackLinearVelocity = feedbackControllerToolbox.getDataVector(endEffector, FEEDBACK, LINEAR_VELOCITY, isEnabled);
         yoFeedForwardLinearVelocity = feedbackControllerToolbox.getDataVector(endEffector, FEEDFORWARD, LINEAR_VELOCITY, isEnabled);
         rateLimitedFeedbackLinearVelocity = feedbackControllerToolbox.getRateLimitedDataVector(endEffector, FEEDBACK, LINEAR_VELOCITY, dt, maximumRate,
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

   public void submitFeedbackControlCommand(PointFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();
      controlBaseFrame = command.getControlBaseFrame();

      inverseDynamicsOutput.set(command.getSpatialAccelerationCommand());

      gains.set(command.getGains());
      linearGainsFrame = command.getLinearGainsFrame();

      command.getBodyFixedPointIncludingFrame(desiredPosition);
      controlFrame.setOffsetToParentToTranslationOnly(desiredPosition);

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
      feedForwardLinearAcceleration.changeFrame(controlFrame);

      desiredLinearAcceleration.setIncludingFrame(proportionalFeedback);
      desiredLinearAcceleration.add(derivativeFeedback);
      desiredLinearAcceleration.add(integralFeedback);
      desiredLinearAcceleration.limitLength(gains.getMaximumFeedback());
      yoFeedbackLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);
      rateLimitedFeedbackLinearAcceleration.update();
      rateLimitedFeedbackLinearAcceleration.getFrameTupleIncludingFrame(desiredLinearAcceleration);

      desiredLinearAcceleration.changeFrame(controlFrame);
      desiredLinearAcceleration.add(feedForwardLinearAcceleration);

      yoDesiredLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);

      addCoriolisAcceleration(desiredLinearAcceleration);

      inverseDynamicsOutput.setLinearAcceleration(controlFrame, desiredLinearAcceleration);
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

      desiredLinearVelocity.changeFrame(controlFrame);
      inverseKinematicsOutput.setLinearVelocity(controlFrame, desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      computeInverseDynamics();
   }

   private final SpatialAccelerationVector achievedSpatialAccelerationVector = new SpatialAccelerationVector();

   @Override
   public void computeAchievedAcceleration()
   {
      spatialAccelerationCalculator.getRelativeAcceleration(base, endEffector, achievedSpatialAccelerationVector);
      achievedSpatialAccelerationVector.changeFrameNoRelativeMotion(controlFrame);
      achievedSpatialAccelerationVector.getLinearPart(achievedLinearAcceleration);
      subtractCoriolisAcceleration(achievedLinearAcceleration);
      yoAchievedLinearAcceleration.setAndMatchFrame(achievedLinearAcceleration);
   }

   /**
    * Computes the feedback term resulting from the error in position:<br>
    * x<sub>FB</sub> = kp * (x<sub>desired</sub> - x<sub>current</sub>)
    * <p>
    * The desired position of the {@code controlFrame} is obtained from {@link #yoDesiredPosition}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentPosition} and {@link #yoErrorPosition}.
    * </p>
    * 
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   private void computeProportionalTerm(FrameVector feedbackTermToPack)
   {
      currentPosition.setToZero(controlFrame);
      currentPosition.changeFrame(worldFrame);
      yoCurrentPosition.set(currentPosition);

      yoDesiredPosition.getFrameTupleIncludingFrame(desiredPosition);

      feedbackTermToPack.setToZero(worldFrame);
      feedbackTermToPack.sub(desiredPosition, currentPosition);
      feedbackTermToPack.limitLength(gains.getMaximumProportionalError());
      yoErrorPosition.set(feedbackTermToPack);

      if (linearGainsFrame != null)
         feedbackTermToPack.changeFrame(linearGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      kp.transform(feedbackTermToPack.getVector());

      feedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the error in linear velocity:<br>
    * x<sub>FB</sub> = kd * (xDot<sub>desired</sub> - xDot<sub>current</sub>)
    * <p>
    * The desired linear velocity of the {@code controlFrame}'s origin relative to the {@code base}
    * is obtained from {@link #yoDesiredLinearVelocity}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentLinearVelocity} and {@link #yoErrorLinearVelocity}.
    * </p>
    * 
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified
    */
   private void computeDerivativeTerm(FrameVector feedbackTermToPack)
   {
      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      currentTwist.getLinearPart(currentLinearVelocity);
      currentLinearVelocity.changeFrame(worldFrame);
      yoCurrentLinearVelocity.set(currentLinearVelocity);

      yoDesiredLinearVelocity.getFrameTupleIncludingFrame(desiredLinearVelocity);

      feedbackTermToPack.setToZero(worldFrame);
      feedbackTermToPack.sub(desiredLinearVelocity, currentLinearVelocity);
      feedbackTermToPack.limitLength(gains.getMaximumDerivativeError());
      yoErrorLinearVelocity.set(feedbackTermToPack);

      if (linearGainsFrame != null)
         feedbackTermToPack.changeFrame(linearGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      kd.transform(feedbackTermToPack.getVector());

      feedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the integrated error in position:<br>
    * x<sub>FB</sub> = ki * &int;<sup>t</sup> (x<sub>desired</sub> - x<sub>current</sub>)
    * <p>
    * The current error in position of the {@code controlFrame} is obtained from
    * {@link #yoErrorPosition}.
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
         feedbackTermToPack.setToZero(controlFrame);
         yoErrorPositionIntegrated.setToZero();
         return;
      }

      yoErrorPosition.getFrameTupleIncludingFrame(feedbackTermToPack);
      feedbackTermToPack.scale(dt);
      feedbackTermToPack.add(yoErrorPositionIntegrated.getFrameTuple());
      feedbackTermToPack.limitLength(maximumIntegralError);
      yoErrorPositionIntegrated.set(feedbackTermToPack);

      if (linearGainsFrame != null)
         feedbackTermToPack.changeFrame(linearGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      ki.transform(feedbackTermToPack.getVector());

      feedbackTermToPack.changeFrame(controlFrame);
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
   private void addCoriolisAcceleration(FrameVector linearAccelerationToModify)
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
   private void subtractCoriolisAcceleration(FrameVector linearAccelerationToModify)
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
   public SpatialAccelerationCommand getVirtualModelControlOutput()
   {
      return getInverseDynamicsOutput();
   }
}
