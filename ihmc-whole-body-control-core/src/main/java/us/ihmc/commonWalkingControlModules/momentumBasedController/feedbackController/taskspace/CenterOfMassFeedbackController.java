package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerException;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.mecano.algorithms.CentroidalMomentumRateCalculator;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.wholeBodyControlCore.pidGains.PID3DGainsBasics;
import us.ihmc.robotics.math.filters.RateLimitedYoMutableFrameVector3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.*;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.*;

public class CenterOfMassFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String shortName = "CenterOfMassFBController";

   private final YoBoolean isEnabled;

   private final FramePoint3DBasics yoDesiredPosition;
   private final FramePoint3DBasics yoCurrentPosition;
   private final FrameVector3DBasics yoErrorPosition;

   private final FrameVector3DBasics yoErrorPositionIntegrated;

   private final FrameVector3DBasics yoDesiredLinearVelocity;
   private final FrameVector3DBasics yoCurrentLinearVelocity;
   private final FrameVector3DBasics yoErrorLinearVelocity;
   private final FeedbackControllerSettings.FilterVector3D linearVelocityErrorFilter;
   private final FrameVector3DBasics yoFeedForwardLinearVelocity;
   private final FrameVector3DBasics yoFeedbackLinearVelocity;
   private final RateLimitedYoMutableFrameVector3D rateLimitedFeedbackLinearVelocity;

   private final FrameVector3DBasics yoDesiredLinearAcceleration;
   private final FrameVector3DBasics yoFeedForwardLinearAcceleration;
   private final FrameVector3DBasics yoFeedbackLinearAcceleration;
   private final RateLimitedYoMutableFrameVector3D rateLimitedFeedbackLinearAcceleration;
   private final FrameVector3DBasics yoAchievedLinearAcceleration;

   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearVelocity = new FrameVector3D();

   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();

   private final MomentumRateCommand inverseDynamicsOutput = new MomentumRateCommand();
   private final MomentumCommand inverseKinematicsOutput = new MomentumCommand();
   private final MomentumRateCommand virtualModelControlOutput = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final PID3DGainsBasics gains;
   private final Matrix3D tempGainMatrix = new Matrix3D();

   private ReferenceFrame centerOfMassFrame;
   private CentroidalMomentumRateCalculator centroidalMomentumHandler;

   private final double dt;
   private final DoubleProvider totalMassProvider;
   private final boolean computeIntegralTerm;

   public CenterOfMassFeedbackController(WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox fbToolbox, YoRegistry parentRegistry)
   {
      centerOfMassFrame = toolbox.getCenterOfMassFrame();
      centroidalMomentumHandler = toolbox.getCentroidalMomentumRateCalculator();

      totalMassProvider = toolbox.getTotalMassProvider();
      FeedbackControllerSettings settings = toolbox.getFeedbackControllerSettings();
      if (settings != null)
         computeIntegralTerm = settings.enableIntegralTerm();
      else
         computeIntegralTerm = true;

      dt = toolbox.getControlDT();
      gains = fbToolbox.getOrCreateCenterOfMassGains(computeIntegralTerm, true);
      DoubleProvider maximumRate = gains.getMaximumFeedbackRateProvider();

      isEnabled = new YoBoolean("is" + shortName + "Enabled", fbToolbox.getRegistry());
      isEnabled.set(false);

      yoDesiredPosition = fbToolbox.getOrCreateCenterOfMassPositionData(DESIRED, isEnabled, true);
      yoCurrentPosition = fbToolbox.getOrCreateCenterOfMassPositionData(CURRENT, isEnabled, true);
      yoErrorPosition = fbToolbox.getOrCreateCenterOfMassVectorData(ERROR, POSITION, isEnabled, false);

      yoErrorPositionIntegrated = computeIntegralTerm ? fbToolbox.getOrCreateCenterOfMassVectorData(ERROR_INTEGRATED, POSITION, isEnabled, false) : null;

      yoDesiredLinearVelocity = fbToolbox.getOrCreateCenterOfMassVectorData(DESIRED, LINEAR_VELOCITY, isEnabled, true);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentLinearVelocity = fbToolbox.getOrCreateCenterOfMassVectorData(CURRENT, LINEAR_VELOCITY, isEnabled, true);
         yoErrorLinearVelocity = fbToolbox.getOrCreateCenterOfMassVectorData(ERROR, LINEAR_VELOCITY, isEnabled, false);
         linearVelocityErrorFilter = fbToolbox.getOrCreateCenterOfMassLinearVelocityErrorFilter(dt);

         yoDesiredLinearAcceleration = fbToolbox.getOrCreateCenterOfMassVectorData(DESIRED, LINEAR_ACCELERATION, isEnabled, true);
         yoFeedForwardLinearAcceleration = fbToolbox.getOrCreateCenterOfMassVectorData(FEEDFORWARD, LINEAR_ACCELERATION, isEnabled, false);
         yoFeedbackLinearAcceleration = fbToolbox.getOrCreateCenterOfMassVectorData(FEEDBACK, LINEAR_ACCELERATION, isEnabled, false);
         rateLimitedFeedbackLinearAcceleration = fbToolbox.getOrCreateCenterOfMassRateLimitedVectorData(FEEDBACK,
                                                                                                        LINEAR_ACCELERATION,
                                                                                                        dt,
                                                                                                        maximumRate,
                                                                                                        isEnabled,
                                                                                                        false);
         yoAchievedLinearAcceleration = fbToolbox.getOrCreateCenterOfMassVectorData(ACHIEVED, LINEAR_ACCELERATION, isEnabled, true);
      }
      else
      {
         yoCurrentLinearVelocity = null;
         yoErrorLinearVelocity = null;
         linearVelocityErrorFilter = null;

         yoDesiredLinearAcceleration = null;
         yoFeedForwardLinearAcceleration = null;
         yoFeedbackLinearAcceleration = null;
         rateLimitedFeedbackLinearAcceleration = null;
         yoAchievedLinearAcceleration = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackLinearVelocity = fbToolbox.getOrCreateCenterOfMassVectorData(FEEDBACK, LINEAR_VELOCITY, isEnabled, false);
         yoFeedForwardLinearVelocity = fbToolbox.getOrCreateCenterOfMassVectorData(FEEDFORWARD, LINEAR_VELOCITY, isEnabled, false);
         rateLimitedFeedbackLinearVelocity = fbToolbox.getOrCreateCenterOfMassRateLimitedVectorData(FEEDBACK,
                                                                                                    LINEAR_VELOCITY,
                                                                                                    dt,
                                                                                                    maximumRate,
                                                                                                    isEnabled,
                                                                                                    false);
      }
      else
      {
         yoFeedbackLinearVelocity = null;
         yoFeedForwardLinearVelocity = null;
         rateLimitedFeedbackLinearVelocity = null;
      }
   }

   public void submitFeedbackControlCommand(CenterOfMassFeedbackControlCommand command)
   {
      inverseDynamicsOutput.set(command.getMomentumRateCommand());

      gains.set(command.getGains());
      command.getMomentumRateCommand().getSelectionMatrix(selectionMatrix);

      yoDesiredPosition.setIncludingFrame(command.getReferencePosition());
      yoDesiredLinearVelocity.setIncludingFrame(command.getReferenceLinearVelocity());
      yoDesiredLinearVelocity.checkReferenceFrameMatch(yoDesiredPosition);
      if (yoFeedForwardLinearVelocity != null)
      {
         yoFeedForwardLinearVelocity.setIncludingFrame(command.getReferenceLinearVelocity());
         yoFeedForwardLinearVelocity.checkReferenceFrameMatch(yoDesiredPosition);
      }
      if (yoFeedForwardLinearAcceleration != null)
      {
         yoFeedForwardLinearAcceleration.setIncludingFrame(command.getReferenceLinearAcceleration());
         yoFeedForwardLinearAcceleration.checkReferenceFrameMatch(yoDesiredPosition);
      }
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
      if (linearVelocityErrorFilter != null)
         linearVelocityErrorFilter.reset();
      if (yoErrorPositionIntegrated != null)
         yoErrorPositionIntegrated.setToZero(worldFrame);
   }

   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();
   private final FrameVector3D integralFeedback = new FrameVector3D();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);
      feedForwardLinearAcceleration.setIncludingFrame(yoFeedForwardLinearAcceleration);
      feedForwardLinearAcceleration.changeFrame(centerOfMassFrame);

      desiredLinearAcceleration.setIncludingFrame(proportionalFeedback);
      desiredLinearAcceleration.add(derivativeFeedback);
      desiredLinearAcceleration.add(integralFeedback);
      desiredLinearAcceleration.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackLinearAcceleration.setIncludingFrame(desiredLinearAcceleration);
      yoFeedbackLinearAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackLinearAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackLinearAcceleration.update();
      desiredLinearAcceleration.setIncludingFrame(rateLimitedFeedbackLinearAcceleration);

      desiredLinearAcceleration.changeFrame(centerOfMassFrame);
      desiredLinearAcceleration.add(feedForwardLinearAcceleration);

      yoDesiredLinearAcceleration.setIncludingFrame(desiredLinearAcceleration);
      yoDesiredLinearAcceleration.changeFrame(trajectoryFrame);

      desiredLinearAcceleration.scale(totalMassProvider.getValue());
      desiredLinearAcceleration.changeFrame(worldFrame);
      inverseDynamicsOutput.setLinearMomentumRate(desiredLinearAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      feedForwardLinearVelocity.setIncludingFrame(yoFeedForwardLinearVelocity);
      computeProportionalTerm(proportionalFeedback);
      computeIntegralTerm(integralFeedback);

      desiredLinearVelocity.setIncludingFrame(proportionalFeedback);
      desiredLinearVelocity.add(integralFeedback);
      desiredLinearVelocity.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackLinearVelocity.setIncludingFrame(desiredLinearVelocity);
      yoFeedbackLinearVelocity.changeFrame(trajectoryFrame);
      rateLimitedFeedbackLinearVelocity.changeFrame(trajectoryFrame);
      rateLimitedFeedbackLinearVelocity.update();
      desiredLinearVelocity.setIncludingFrame(rateLimitedFeedbackLinearVelocity);

      desiredLinearVelocity.add(feedForwardLinearVelocity);

      yoDesiredLinearVelocity.setIncludingFrame(desiredLinearVelocity);
      yoDesiredLinearVelocity.changeFrame(trajectoryFrame);

      desiredLinearVelocity.scale(totalMassProvider.getValue());
      desiredLinearVelocity.changeFrame(worldFrame);
      inverseKinematicsOutput.setLinearMomentum(desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      if (!isEnabled())
         return;

      virtualModelControlOutput.set(inverseDynamicsOutput);
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);
      feedForwardLinearAcceleration.setIncludingFrame(yoFeedForwardLinearAcceleration);
      feedForwardLinearAcceleration.changeFrame(centerOfMassFrame);

      desiredLinearAcceleration.setIncludingFrame(proportionalFeedback);
      desiredLinearAcceleration.add(derivativeFeedback);
      desiredLinearAcceleration.add(integralFeedback);
      desiredLinearAcceleration.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackLinearAcceleration.setIncludingFrame(desiredLinearAcceleration);
      yoFeedbackLinearAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackLinearAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackLinearAcceleration.update();
      desiredLinearAcceleration.setIncludingFrame(rateLimitedFeedbackLinearAcceleration);

      desiredLinearAcceleration.changeFrame(centerOfMassFrame);
      desiredLinearAcceleration.add(feedForwardLinearAcceleration);

      yoDesiredLinearAcceleration.setIncludingFrame(desiredLinearAcceleration);
      yoDesiredLinearAcceleration.changeFrame(trajectoryFrame);

      desiredLinearAcceleration.scale(totalMassProvider.getValue());
      desiredLinearAcceleration.changeFrame(worldFrame);
      virtualModelControlOutput.setLinearMomentumRate(desiredLinearAcceleration);
   }

   @Override
   public void computeAchievedAcceleration()
   {
      SpatialForceReadOnly achievedMomentumRate = centroidalMomentumHandler.getMomentumRate();
      yoAchievedLinearAcceleration.setIncludingFrame(achievedMomentumRate.getLinearPart());
      yoAchievedLinearAcceleration.scale(1.0 / totalMassProvider.getValue());
      yoAchievedLinearAcceleration.changeFrame(yoDesiredPosition.getReferenceFrame());
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
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      yoCurrentPosition.setToZero(centerOfMassFrame);
      yoCurrentPosition.changeFrame(trajectoryFrame);

      feedbackTermToPack.setToZero(trajectoryFrame);
      feedbackTermToPack.sub(yoDesiredPosition, yoCurrentPosition);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(gains.getMaximumProportionalError());

      yoErrorPosition.setIncludingFrame(feedbackTermToPack);
      yoErrorPosition.changeFrame(trajectoryFrame);

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
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      yoCurrentLinearVelocity.setIncludingFrame(centroidalMomentumHandler.getCenterOfMassVelocity());
      yoCurrentLinearVelocity.changeFrame(trajectoryFrame);

      feedbackTermToPack.setToZero(trajectoryFrame);
      feedbackTermToPack.sub(yoDesiredLinearVelocity, yoCurrentLinearVelocity);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(gains.getMaximumDerivativeError());
      // TODO: there is an inconsistency here between this feedback controller and the point feedback controller:
      // The point feedback controller sets this value after rate limiting.
      yoErrorLinearVelocity.setIncludingFrame(feedbackTermToPack);
      yoErrorLinearVelocity.changeFrame(trajectoryFrame);
      if (linearVelocityErrorFilter != null)
         linearVelocityErrorFilter.apply(feedbackTermToPack, feedbackTermToPack);

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
      if (!computeIntegralTerm)
      {
         feedbackTermToPack.setToZero(centerOfMassFrame);
         return;
      }

      double maximumIntegralError = gains.getMaximumIntegralError();
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      if (maximumIntegralError < 1.0e-5)
      {
         feedbackTermToPack.setToZero(centerOfMassFrame);
         yoErrorPositionIntegrated.setToZero(trajectoryFrame);
         return;
      }

      // If the trajectory frame changed reset the integration.
      if (yoErrorPositionIntegrated.getReferenceFrame() != trajectoryFrame)
      {
         yoErrorPositionIntegrated.setToZero(trajectoryFrame);
      }

      feedbackTermToPack.setIncludingFrame(yoErrorPosition);
      feedbackTermToPack.scale(dt);
      feedbackTermToPack.add(yoErrorPositionIntegrated);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(maximumIntegralError);
      yoErrorPositionIntegrated.setIncludingFrame(feedbackTermToPack);
      yoErrorPositionIntegrated.changeFrame(trajectoryFrame);

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
         throw new FeedbackControllerException("This controller is disabled.");
      return inverseDynamicsOutput;
   }

   @Override
   public MomentumCommand getInverseKinematicsOutput()
   {
      if (!isEnabled())
         throw new FeedbackControllerException("This controller is disabled.");
      return inverseKinematicsOutput;
   }

   @Override
   public MomentumRateCommand getVirtualModelControlOutput()
   {
      if (!isEnabled())
         throw new FeedbackControllerException("This controller is disabled.");
      return virtualModelControlOutput;
   }
}
